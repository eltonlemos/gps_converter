import numpy as np 
np.set_printoptions(suppress=True)
gps_origin = [0,0,0]

p1_gps = [4,-4, (3/4)*np.pi]

p2_gps = [1,0, (2/4)*np.pi]

p1 = np.array(p1_gps)
p2 = np.array(p2_gps)

def coordinate_transform(global_local_origin, global_local_point):
    x = global_local_point[0] - global_local_origin[0]
    y = global_local_point[1] - global_local_origin[1]
    theta = global_local_origin[2]
    x_local = x*np.cos(theta) + y*np.sin(theta)
    y_local = -x*np.sin(theta) + y*np.cos(theta)
    yaw_local = global_local_point[2] - global_local_origin[2]
    return [x_local, y_local, yaw_local]


p1 = coordinate_transform(p1, p2)
print(p1)
