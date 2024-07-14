import numpy as np 

gps_origin = [0,0,0]

p1_gps = [1,-1,np.pi/2]

p2_gps = [1,1,np.pi/2]

p1 = np.array(p1_gps)
p2 = np.array(p2_gps)

def coordinate_transform(frame_origin_xyt,point_gps_xyt):
    x = point_gps_xyt[0] - frame_origin_xyt[0]
    y = point_gps_xyt[1] - frame_origin_xyt[1]
    
    yaw_temp = frame_origin_xyt[2]
    
    x_local = x*np.cos(-yaw_temp) - y*np.sin(-yaw_temp)
    y_local = x*np.sin(-yaw_temp) + y*np.cos(-yaw_temp)

    yaw_local = point_gps_xyt[2] - frame_origin_xyt[2]
    print(x_local,y_local,yaw_local)
    return [x_local,y_local,yaw_local]

p1_local = coordinate_transform(p1_gps,p2_gps)