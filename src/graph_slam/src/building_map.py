#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from bresenham import bresenham


class Viewer:
    """
    Use matplotlib to show costmap and plot cmd_vel predicted locations.
    """
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        plt.ion()
        plt.show()

    def update(self):
        """
        Update matplotlib ax.
        """
        self._plot_scan_map_high_resolution()
        self._plot_robot()
        plt.pause(0.01)
        self.fig.canvas.draw()
        


    def _plot_scan_map_high_resolution(self):
        
        global MAP, SIZE
        self.ax.cla() 
        self.ax.imshow(X=MAP[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax.grid(which='both', color='gray')

    def _plot_robot(self):
        """
        Plot robot with orientation (assuming the location is at the center).
        """

        global SIZE
        center = int(SIZE/2)

        try:
            x = center
            y = center
            self.ax.plot(x, y, marker='o', markersize=10, linestyle='None')
        except Exception as err:
            rospy.loginfo(err)

    



def cb_scan(msg):

    global MAP, SIZE, LOG_odd_occ, LOG_odd_free
    center = int(SIZE/2)
    resolution = int(2000 / SIZE)

    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            x = dist * math.cos(ind * math.pi / 725)
            y = dist * math.sin(ind * math.pi / 725)

            px = int(100*x/resolution) + center
            py = int(100*y/resolution) + center
            
            MAP[px,py] += LOG_odd_occ

            pixel_free = list(bresenham(center, center, px, py))
            for i in range(len(pixel_free)-2):
                MAP[pixel_free[i+1]] -= LOG_odd_free

            
            



if __name__ == "__main__":
    
    # -- lookup table
    RAD_TABLE_COS = []
    RAD_TABLE_SIN = []
    for deg in range(360):
        RAD_TABLE_COS.append(
            int(1000*math.cos(deg/360 *math.pi))
        )
        RAD_TABLE_SIN.append(
            int(1000*math.sin(deg/360 *math.pi))
        )



    # -- global vars
    RESOLUTION = 5
    SIZE = int(2000 / RESOLUTION)
    COST = 0
    MAP = np.zeros((SIZE,SIZE))

    LOG_odd_occ = 1
    LOG_odd_free = 1


    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    
    # -- ros function
    rospy.Subscriber(name='/solamr_1/scan_lidar', data_class=LaserScan, callback=cb_scan)

    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=0.5)
    

    while not rospy.is_shutdown():
        
        viewer.update()
        rate.sleep()
