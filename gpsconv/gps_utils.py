from osgeo import gdal
import numpy as np
import matplotlib.pyplot as plt
import pyproj
import pyproj.transformer
from tqdm import tqdm
from scipy.spatial import KDTree
import math

np.set_printoptions(suppress=True)


def pixel2latlon(x, y, gt):
    """Returns lat lon from pixel
    Args:
    x, y: pixel coordinates
    gt: gdal geotransform

    Returns:
    lat, lon: latitude and longitude
    """
    gt_0, gt_1, gt_2, gt_3, gt_4, gt_5 = gt

    lon = gt_0 + x * gt_1 + y * gt_2
    lat = gt_3 + x * gt_4 + y * gt_5
    return lat, lon

def latlon2pixel(lat, lon, gt):
    """Returns pixel from lat lon
    Args:
    lat, lon: latitude and longitude
    ds: gdal dataset

    Returns:
    pixel_x, pixel_y: pixel coordinates
    """
    forward_transform = gt
    reverse_transform = gdal.InvGeoTransform(forward_transform)
    pixel_coord = gdal.ApplyGeoTransform(reverse_transform, lon, lat)
    pixel_x = math.floor(pixel_coord[0])
    pixel_y = math.floor(pixel_coord[1])

    return pixel_y, pixel_x


def printBounds(array):
    """
    Print the X, Y, Z bounds of the array

    Args:
    array: numpy array of shape (N,3)

    Returns:
    None
    """
    x_min = np.min(array[:,0])
    x_max = np.max(array[:,0])
    y_min = np.min(array[:,1])
    y_max = np.max(array[:,1])
    z_min = np.min(array[:,2])
    z_max = np.max(array[:,2])

    print(f"X: {x_min} - {x_max}")
    print(f"Y: {y_min} - {y_max}")
    print(f"Z: {z_min} - {z_max}")


def geodetic2ECEF( lati, longi, alti ):
    """ Converts geodetic coordinates to ECEF coordinates

    Args:
    lati: latitude in degrees
    longi: longitude in degrees
    alti: altitude in meters (mean sea level)

    Returns:
    X, Y, Z: ECEF coordinates
    """
    # Adopted from https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
    phi = lati / 180. * np.pi
    lambada = longi / 180. * np.pi
    h = alti

    #N = 6371000 #in meters
    e = 0.081819191 #earth ecentricity
    q = np.sin( phi )
    N = 6378137.0 / np.sqrt( 1 - e*e * q*q )
    X = (N + h) * np.cos( phi ) * np.cos( lambada )
    Y = (N + h) * np.cos( phi ) * np.sin( lambada )
    Z = (N*(1-e*e) + h) * np.sin( phi )

    return X,Y,Z

def ecef2Geodetic(x,y,z):
    '''
    https://gis.stackexchange.com/a/292635
    Function to convert xyz ECEF to llh
    convert cartesian coordinate into geographic coordinate
    ellipsoid definition: WGS84
      a= 6,378,137m
      f= 1/298.257

    Args
      x: coordinate X meters
      y: coordinate y meters
      z: coordinate z meters

    Returns
      lat: latitude rad
      lon: longitude rad
      h: height meters
    '''
    # --- WGS84 constants
    a = 6378137.0
    f = 1.0 / 298.257223563
    # --- derived constants
    b = a - f*a
    e = math.sqrt(math.pow(a,2.0)-math.pow(b,2.0))/a
    clambda = math.atan2(y,x)
    p = math.sqrt(pow(x,2.0)+pow(y,2))
    h_old = 0.0
    # first guess with h=0 meters
    theta = math.atan2(z,p*(1.0-math.pow(e,2.0)))
    cs = math.cos(theta)
    sn = math.sin(theta)
    N = math.pow(a,2.0)/math.sqrt(math.pow(a*cs,2.0)+math.pow(b*sn,2.0))
    h = p/cs - N
    while abs(h-h_old) > 1.0e-6:
        h_old = h
        theta = math.atan2(z,p*(1.0-math.pow(e,2.0)*N/(N+h)))
        cs = math.cos(theta)
        sn = math.sin(theta)
        N = math.pow(a,2.0)/math.sqrt(math.pow(a*cs,2.0)+math.pow(b*sn,2.0))
        h = p/cs - N

    return np.degrees(theta), np.degrees(clambda), h  

def ecef2ENU( lati_r, longi_r ):
    """ Computes a matrix_3x3 which transforms a ecef diff-point to ENU (East-North-Up)
    
    Args:
    lati_r: latitude in degrees
    longi_r: longitude in degrees

    Returns:
    T_enu_ecef: 3x3 transformation matrix from ECEF to ENU
    
    Reference:
    Adopted from https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
    """

    phi = lati_r / 180. * np.pi
    lambada = longi_r / 180. * np.pi

    cp = np.cos( phi ) #cos(phi)
    sp = np.sin( phi ) #cos(phi)
    cl = np.cos( lambada )
    sl = np.sin( lambada )

    T = np.zeros( (3,3), dtype='float64' )
    T[0,0] = -sl
    T[0,1] = cl
    T[0,2] = 0

    T[1,0] = -sp * cl
    T[1,1] = -sp * sl
    T[1,2] = cp

    T[2,0] = cp * cl
    T[2,1] = cp * sl
    T[2,2] = sp

    T_enu_ecef = T
    return T_enu_ecef

def enu2ECEF( points, lati_r, longi_r, ref_point ):
    """
    Converts ENU points to ECEF points

    Args:
    points: local ENU points (Nx3)
    lati_r: latitude in degrees
    longi_r: longitude in degrees
    ref_point: reference point in ECEF coordinates

    """
    T = ecef2ENU( lati_r, longi_r )
    T = np.linalg.inv(T)
    p = np.dot(T, points.T).T
    if type(ref_point) != np.ndarray:
        ref_point = np.array(ref_point)

    p = p + ref_point

    return p




def getData(tiff_file=None,max_extent=True,NW:list=None,SE:list=None):
    if tiff_file is None:
        raise ValueError("Please provide a tiff file")
    
    dataset = gdal.Open(tiff_file, gdal.GA_ReadOnly)

    print("Driver: {}/{}".format(dataset.GetDriver().ShortName,
                                dataset.GetDriver().LongName))
    print("Size is {} x {} x {}".format(dataset.RasterXSize,
                                        dataset.RasterYSize,
                                        dataset.RasterCount))
    print("Projection is {}".format(dataset.GetProjection()))
    geotransform = dataset.GetGeoTransform()
    if geotransform:
        print("Origin = ({}, {})".format(geotransform[0], geotransform[3]))
        print("Pixel Size = ({}, {})".format(geotransform[1], geotransform[5]))


    band = dataset.GetRasterBand(1)
    print("Band Type={}".format(gdal.GetDataTypeName(band.DataType)))

    min = band.GetMinimum()
    max = band.GetMaximum()
    if not min or not max:
        (min,max) = band.ComputeRasterMinMax(True)
    print("Min={:.3f}, Max={:.3f}".format(min,max))

    if band.GetOverviewCount() > 0:
        print("Band has {} overviews\n".format(band.GetOverviewCount()))

    if band.GetRasterColorTable():
        print("Band has a color table with {} entries\n".format(band.GetRasterColorTable().GetCount()))

    ele = band.ReadAsArray()
    mask = ele < min
    ele_masked = np.copy(ele)
    ele_masked[mask] = min
    ele = np.copy(ele_masked)
    # ele_norm = (ele_masked - min) / (max - min)

    mid_row = ele.shape[0] // 2
    mid_col = ele.shape[1] // 2

    im_extent = [
        [0,0],
        [0, mid_col],
        [0, 2*mid_col - 1],
        [mid_row, 0],
        [mid_row, mid_col],
        [mid_row, 2*mid_col - 1],
        [2*mid_row - 1, 0],
        [2*mid_row - 1, mid_col],
        [2*mid_row - 1, 2*mid_col - 1]
    ]

    latlon_extent = []
    for i in im_extent:
        latlon_extent.append(i)
    if max_extent == True:
        NW_lat, NW_lon = pixel2latlon(latlon_extent[0][0], latlon_extent[0][1], geotransform)
        C_lat, C_lon = pixel2latlon(latlon_extent[4][0], latlon_extent[4][1], geotransform)
        SE_lat, SE_lon = pixel2latlon(latlon_extent[8][0], latlon_extent[8][1], geotransform)
        C_interp_lat, C_interp_lon = (NW_lat + SE_lat) / 2, (NW_lon + SE_lon) / 2

        print("="*50)
        print(f"WGS84 Coordinates")
        print("="*50)
        print(f"North-West: {NW_lat}, {NW_lon}")
        print(f"Center: {C_lat}, {C_lon}")
        print(f"South-East: {SE_lat}, {SE_lon}")


        # print(f"NW: {NW_lat}, {NW_lon}, {ele[im_extent[0][0], im_extent[0][1]]}")
        # print(f"C: {C_lat}, {C_lon}, {ele[im_extent[4][0], im_extent[4][1]]}")
        # print(f"C_i: {C_interp_lat}, {C_interp_lon}")
        # print(f"C Error: {C_lat-C_interp_lat}, {C_lon-C_interp_lon}")
        # print(f"SE: {SE_lat}, {SE_lon}, {ele[im_extent[8][0], im_extent[8][1]]}")

        NW_px, NW_py = latlon2pixel(NW_lat, NW_lon, geotransform)
        SE_px, SE_py = latlon2pixel(SE_lat, SE_lon, geotransform)
        C_px, C_py = latlon2pixel(C_lat, C_lon, geotransform)

        print("\n")
        print("="*50)
        print(f"Taking raw latlon and converting to pixel")
        print("="*50)
        print(f"Image: {ele_masked.shape}")
        print(f"NW px: {NW_py}, {NW_px}")
        print(f"C px: {C_py}, {C_px}")
        print(f"SE px: {SE_py}, {SE_px}")

    else:
        NW_lat = NW[0]
        NW_lon = NW[1]
        SE_lat = SE[0]
        SE_lon = SE[1]
        C_lat = (NW_lat + SE_lat) / 2
        C_lon = (NW_lon + SE_lon) / 2
        C_interp_lat, C_interp_lon = C_lat, C_lon

        NW_px, NW_py = latlon2pixel(NW_lat, NW_lon, dataset)
        SE_px, SE_py = latlon2pixel(SE_lat, SE_lon, dataset)
        C_px, C_py = latlon2pixel(C_lat, C_lon, dataset)

        # print(f"NW: {NW_lat}, {NW_lon}, {ele[NW_px, NW_py]}")
        # print(f"C: {C_lat}, {C_lon}, {ele[C_px, C_py]}")
        # print(f"SE: {SE_lat}, {SE_lon}, {ele[SE_px, SE_py]}")
        # print(f"C Error: {C_lat-C_interp_lat}, {C_lon-C_interp_lon}")

    data = {
        "ele_masked": ele_masked,
        "gt": geotransform,
        "im_extent": im_extent,
        "latlon_extent": latlon_extent,
        "NW": {"lat": NW_lat, "lon": NW_lon},
        "C": {"lat": C_lat, "lon": C_lon},
        "SE": {"lat": SE_lat, "lon": SE_lon},
        "C_interp": {"lat": C_interp_lat, "lon": C_interp_lon},
    }

    return data

def runSample(tiff=None):
    """
    Prints data for validation

    Args:
    tiff_location: location of the tif file

    Returns:
    None
    """

    if tiff is None:
        raise ValueError("Please provide a tiff file")
    
    print("="*50)
    print(f"Sample for {tiff}")
    print("="*50)
    data_dict = getData(tiff_file=tiff,max_extent=True)

    ele = data_dict["ele_masked"]
    im_extent = data_dict["im_extent"]
    NW_lat, NW_lon = data_dict["NW"]["lat"], data_dict["NW"]["lon"]
    C_lat, C_lon = data_dict["C"]["lat"], data_dict["C"]["lon"]
    SE_lat, SE_lon = data_dict["SE"]["lat"], data_dict["SE"]["lon"]
    gt = data_dict["gt"]

    

    NW_X, NW_Y, NW_Z = geodetic2ECEF(NW_lat, NW_lon, ele[im_extent[0][0], im_extent[0][1]])
    C_X, C_Y, C_Z = geodetic2ECEF(C_lat, C_lon, ele[im_extent[4][0], im_extent[4][1]])
    SE_X, SE_Y, SE_Z = geodetic2ECEF(SE_lat, SE_lon, ele[im_extent[8][0], im_extent[8][1]])

    T = ecef2ENU(C_lat, C_lon)

    X = [NW_X, C_X, SE_X]
    Y = [NW_Y, C_Y, SE_Y]
    Z = [NW_Z, C_Z, SE_Z]

    X = np.array(X)
    Y = np.array(Y)
    Z = np.array(Z)

    delta = np.array([X-C_X,Y-C_Y,Z-C_Z])
    p = np.dot(T,delta).T

    NW_x_local, NW_y_local, NW_z_local = p[0]
    C_x_local, C_y_local, C_z_local = p[1]
    SE_x_local, SE_y_local, SE_z_local = p[2]

    

    # ENU to ECEF
    NW_ecef = enu2ECEF(np.array([NW_x_local, NW_y_local, NW_z_local]), C_lat, C_lon, [C_X, C_Y, C_Z])
    C_ecef = enu2ECEF(np.array([C_x_local, C_y_local, C_z_local]), C_lat, C_lon, [C_X, C_Y, C_Z])
    SE_ecef = enu2ECEF(np.array([SE_x_local, SE_y_local, SE_z_local]), C_lat, C_lon, [C_X, C_Y, C_Z])

    # print(f"NW ECEF: {NW_ecef}")
    # print(f"C ECEF: {C_ecef}")
    # print(f"SE ECEF: {SE_ecef}")
    print("\n")
    print("="*50)
    print(f"ECEF Coordinates")
    print("="*50)
    print(f"North-West ECEF: {NW_ecef}")
    print(f"Center ECEF: {C_ecef}")
    print(f"South-East ECEF: {SE_ecef}")

    # ECEF to LLH
    NW_llh = ecef2Geodetic(*NW_ecef)
    C_llh = ecef2Geodetic(*C_ecef)
    SE_llh = ecef2Geodetic(*SE_ecef)

    print("\n")
    print("="*50)
    print("Converting raw ECEF to WGS84")
    print("="*50)
    print(f"North-West LLH: {NW_llh}")
    print(f"Center LLH: {C_llh}")
    print(f"South-East LLH: {SE_llh}")

    print("\n")
    test_X, test_Y = 0, 0
    test_Z = 0
    print("="*50)
    print(f"Converting local ({test_X},{test_Y})m to ECEF and LLH")
    print("="*50)

    test_ecef = enu2ECEF(np.array([test_X, test_Y, test_Z]), C_lat, C_lon, [C_X, C_Y, C_Z])

    print(f"Test ECEF: {test_ecef}")

    test_llh = ecef2Geodetic(*test_ecef)

    print(f"Test LLH: {test_llh}")

    test_px, test_py = latlon2pixel(test_llh[0], test_llh[1], gt)

    print(f"Test Pixel: {test_py}, {test_px}")

    test_elev = ele[test_py, test_px]

    test_llh_final = (test_llh[0], test_llh[1], test_elev)

    print(f"Test LLH Final: {test_llh_final}")

def validateCoords(coords,NW_coords=None,SE_coords=None,north_hemisphere=True,west_meridian=True):
    '''
    Validate if the coordinates are within the bounds

    Args:
    coords: numpy array of shape (N,3)
    NW_coords: North-West coordinates
    SE_coords: South-East coordinates

    Returns:
    None

    Raises:
    ValueError: If some coordinates are outside the bounds
    '''
    lats = coords[:,0]
    lons = coords[:,1]


    if NW_coords is None or SE_coords is None:
        raise ValueError("Please provide NW and SE coordinates")

    if north_hemisphere:
        lat_mask_1 = lats > NW_coords[0]
        lat_mask_2 = lats < SE_coords[0]
        lat_mask = np.logical_and(lat_mask_1,lat_mask_2)
    else:
        lat_mask_1 = lats < NW_coords[0]
        lat_mask_2 = lats > SE_coords[0]
        lat_mask = np.logical_and(lat_mask_1,lat_mask_2)

    if west_meridian:
        lon_mask_1 = lons < NW_coords[1]
        lon_mask_2 = lons > SE_coords[1]
        lon_mask = np.logical_and(lon_mask_1,lon_mask_2)
    else:
        lon_mask_1 = lons > NW_coords[1]
        lon_mask_2 = lons < SE_coords[1]
        lon_mask = np.logical_and(lon_mask_1,lon_mask_2)

    if np.any(lat_mask) or np.any(lon_mask):
        # print("Some coordinates are outside the bounds")
        raise ValueError("Some coordinates are outside the bounds")
    
    print("All coordinates are within the bounds")

def wgs84_2ENU(coords,C_lat,C_lon,C_alt=0):
    '''
    Converts WGS84 coordinates to ENU coordinates using interpolated center

    Args:
    coords: numpy array of shape (N,3)
    NW_coords: North-West coordinates
    SE_coords: South-East coordinates

    Returns:
    None
    '''
    if type(coords) == list:
        coords = np.array(coords)

    if coords.shape[1] != 3:
        zeroes = np.zeros((coords.shape[0],1))
        coords = np.hstack((coords,zeroes))

    C_X, C_Y, C_Z = geodetic2ECEF(C_lat, C_lon, C_alt)

    T = ecef2ENU(C_lat, C_lon)

    lats = coords[:,0]
    lons = coords[:,1]
    alts = coords[:,2]

    X, Y, Z = geodetic2ECEF(lats, lons, alts)

    delta = np.array([X-C_X,Y-C_Y,Z-C_Z])
    p = np.dot(T,delta).T
    
    return p
    

    
if __name__ == "__main__":

    # All assumptions are for North Hemisphere and West of Prime Meridian

    # Run Sample
    # runSample(tiff="space.tif")

    # Random Coordinates
    UB_NW = [43.01403, -78.80508]
    UB_SE = [42.98984, -78.76478]

    # print(f"\nRandom Coordinates\n", "*"*50)

    # num_points = 50

    # rand_lats = np.random.uniform(UB_NW[0],UB_SE[0],num_points)
    # rand_lons = np.random.uniform(UB_NW[1],UB_SE[1],num_points)
    # rand_alts = np.random.uniform(0,10,num_points)

    # coords = np.vstack((rand_lats,rand_lons,rand_alts)).T

    # wgs84_2ENU(coords,NW_coords=UB_NW,SE_coords=UB_SE)

    # print(f"Davis Hall Coordinates")
    # print("="*50)

    # DAVIS = [43.00300, -78.78732]

    # coords = np.array([DAVIS])

    # wgs84_2ENU(coords,NW_coords=UB_NW,SE_coords=UB_SE)

    runSample(tiff="space.tif")