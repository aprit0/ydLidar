from data import lidar_0
import lidar_to_grid_map as lg

import math
from collections import deque
import matplotlib.pyplot as plt
import numpy as np


def read_variable(laserScan):
    #laserScan: angle, dist
    angles = []
    distances = []
    for point in laserScan:
        if point[1] > 0:
            
            angles.append(float(point[0])*3.1415/180)
            distances.append(float(point[1])/1000)
    angles = np.array(angles)
    distances = np.array(distances)
    return angles, distances

def from_plt_to_xy(point, map_size, xy_resolution):
    #centre = int(round((map_size[1]-map_size[0])/xy_resolution))
    xy_x = point[0]*xy_resolution + map_size[0]
    xy_y = point[1]*xy_resolution + map_size[0]
    return [xy_x,xy_y]

def from_xy_to_plt(point, map_size, xy_resolution):
    #centre = int(round((map_size[1]-map_size[0])/xy_resolution))
    plt_x = int(round((point[0] - map_size[0]) /xy_resolution))
    plt_y = int(round((point[1] - map_size[0]) /xy_resolution))
    return [plt_x,plt_y]
def createLocalDest(dest, map_size, xy_resolution):
    if dest[0] < map_size[1] and dest[0] > map_size[0]:
        if dest[1] < map_size[1] and dest[1] > map_size[0]:
            #is within the map
            return from_xy_to_plt(dest, map_size, xy_resolution), dest
    else:
        #gradient from origin
        m = dest[1]/dest[0]
        #Based off quadrants and unit circle
        #Q1, Q3: m+ | Q2, Q4: m-
        #90deg, 180deg: dest[1] > dest[0] | 0deg, 270deg: dest[1] < dest[0]
        if m > 0 and (dest[1] > dest[0] and dest[1] > 0) or (dest[1] == dest[0] and dest[0] > 0):
            #45-90
            new_dest = [map_size[1]/m, map_size[1]]
            print("dest0||" + str(m))#
        elif m > 0 and (dest[1] > dest[0] and dest[1] < 0):
            #180-225
            new_dest = [map_size[0], map_size[0]*m]
            print("dest0.5||" + str(m))#
        elif m > 0 and (dest[1] < dest[0] and dest[1] > 0) or (dest[1] == dest[0] and dest[0] < 0):
            #0-45
            new_dest = [map_size[1], map_size[1]*m]
            print("dest1||" + str(m))#
        elif m > 0 and (dest[1] < dest[0] and dest[1] < 0) or (dest[1] == dest[0] and dest[0] < 0):
            #225-275
            new_dest = [map_size[0]/m, map_size[0]]
            print("dest1.5||" + str(m))#
        
        elif m < 0 and (dest[1] > dest[0] and abs(m) > 1) or (dest[1] == dest[0] and dest[0] > 0):
            #90-135
            new_dest = [map_size[1]/m, map_size[0]]
            print("dest2||" + str(m))#
        elif m < 0 and (dest[1] > dest[0] and abs(m) < 1) or (dest[1] == dest[0] and dest[0] > 0):
            #135-180
            new_dest = [map_size[0], map_size[0]*m]
            print("dest2.5||" + str(m))#
            
        elif m < 0 and (dest[1] < dest[0] and abs(m) > 1) or (dest[1] == dest[0] and dest[0] > 0):
            #270-315
            new_dest = [map_size[1]/m, map_size[0]]
            print("dest3||" + str(m))#
        else:
            #315-360
            #m < 0 and dest[1] < dest[0] and abs(m) < 1
            new_dest = [map_size[1], map_size[1]*m]
            print("dest3.5||" + str(m))

        #return new_dest
        return from_xy_to_plt(new_dest, map_size, xy_resolution), new_dest
    
from scipy.spatial import distance


import math

def rotate(origin, point, angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return int(round(qx)), int(round(qy))


    
    
def main():
    xy_resolution = 0.02
    ang, dist = read_variable(lidar_0)
    ox = np.sin(ang) * dist
    oy = np.cos(ang) * dist
    
    map_size = [-10,10]
    
    occupancy_map, min_x, max_x, min_y, max_y, xy_resolution,center_x,clearPoints = lg.generate_ray_casting_grid_map(
        ox, oy, xy_resolution, True,map_size = map_size)
    
    print(min_x,min_y)
    print(max_x,max_y)
    print(center_x,center_x)
    xy_res = np.array(occupancy_map).shape
    fig = plt.figure(1,figsize=(10, 10))
    rows = 1
    columns = 3
    
    fig.add_subplot(rows, columns, 1)
    plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
    plt.axis("equal")
    plt.plot(0.0, 0.0, "ob")
    plt.gca().set_aspect("equal", "box")
    bottom, top = plt.ylim()  # return the current y-lim
    print(bottom, top)
    plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
    plt.grid(True)
    
    fig.add_subplot(rows, columns, 2)
    plt.imshow(occupancy_map, cmap="PiYG_r")
    print(xy_res[0], xy_res[1])
    plt.xlabel("-10,10")
    plt.plot(500, 500, 'yo') 
    plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)    
    plt.colorbar()
    #plt.show()

    #find the closest green point to the destination
    destination = [15,10]
    new_dest = createLocalDest(destination, map_size, xy_resolution)
    print(new_dest)
    print(len(clearPoints.keys()))
    #collect all green pairs
    #collectClearPairs(occupancy_map)
    #print(new_dest[0].shape, clearPoints.keys().shape)
    #print(closest_node(new_dest[0], clearPoints.keys()))
    
    #plt.show()
    XA = np.asarray(new_dest[0]).reshape(-1,2)
    XB = np.asarray(list(clearPoints.keys())).reshape(-1,2)
    print(XA.shape)
    print(XB.shape)
    print((distance.cdist(XA, XB)).argmin())
    print(XB[3717])
    XB_new = rotate([500,500], XB[3717], math.radians(90))
    XA_new = rotate([500,500], new_dest[0], math.radians(90))
    print(XB_new)
    x = []
    y = []
    for i in list(clearPoints.keys()):
        x.append(i[0])
        y.append(i[1])
    #plt.scatter(x,y)
    fig.add_subplot(rows, columns, 3)
    plt.imshow(occupancy_map, cmap="PiYG_r")

    plt.xlabel("-10,10")
    #plt.plot(500, 500, 'yo') 
    plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)    
    plt.plot((500,XB_new[0]),(500,XB_new[1]), label='lineplots', color='b')
    plt.plot((XB_new[0],XA_new[0]),(XB_new[1],XA_new[1]), label='lineplots', color='black')
    #.scatter(XA_new[0],XA_new[1] , color="black") 
    plt.show()
if __name__ == '__main__':
    main()