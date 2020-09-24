#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan


class Viewer:
    """
    Use matplotlib to show costmap and plot cmd_vel predicted locations.
    """
    def __init__(self):
        self.fig, self.ax = plt.subplots(2,1)
        self.ax1 = self.ax[0]
        self.ax2 = self.ax[1]
        plt.ion()
        plt.show()

    def update(self):
        """
        Update matplotlib ax.
        """
        self._plot_scan_map_high_resolution()
        self._plot_scan_map_low_resolution()
        self._plot_robot()
        self.fig.canvas.draw()
        plt.pause(0.001)

        # print(MAP_1_list)

    def _plot_scan_map_high_resolution(self):
        
        global MAP_1
        self.ax1.cla() 
        self.ax1.imshow(X=MAP_1[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax1.grid(which='both', color='grey')
        MAP_1 = np.zeros((400,400))

    def _plot_scan_map_low_resolution(self):
        
        global MAP_2
        self.ax2.cla() 
        self.ax2.imshow(X=MAP_2[::-1], cmap="gray", origin="lower")  # use img to show costmap
        self.ax2.grid(which='both', color='grey')
        MAP_2 = np.zeros((40,40))

    def _plot_costmap(self):
        """
        Plot costmap.
        """
        self.ax.cla()
        self.ax.imshow(X=DATA[::-1], cmap="jet")  # use img to show costmap
        self.ax.grid(which='both', color='grey')

    def _plot_robot(self):
        """
        Plot robot with orientation (assuming the location is at the center).
        """
        try:
            x = 200
            y = 200
            self.ax1.plot(x, y, marker='o', markersize=10, linestyle='None')
        except Exception as err:
            rospy.loginfo(err)

    
# --

def cb_scan(msg):

    global MAP_1, MAP_2, MAP_1_list, MAP_2_list

    MAP_1_list = []
    MAP_2_list = []
    for ind, dist in enumerate(msg.ranges):
        if dist >= msg.range_min and dist <= msg.range_max:
            
            x = dist * math.cos(ind * math.pi / 725)
            y = dist * math.sin(ind * math.pi / 725)

            px = int(100*x/5) + 200
            py = int(100*y/5) + 200
            MAP_1[px,py] = -1

            px = int(100*x/50) + 20
            py = int(100*y/50) + 20
            MAP_2[px,py] = -1
            

    for row in range(MAP_1.shape[0]):
        for col in range(MAP_1.shape[1]):

            if MAP_1[row, col] == -1:
                MAP_1_list.append([
                    row, col
                ])


    for row in range(MAP_2.shape[0]):
        for col in range(MAP_2.shape[1]):

            if MAP_2[row, col] == -1:
                MAP_2_list.append([
                    row, col
                ])



if __name__ == "__main__":

    # -- global vars
    MAP_1 = np.zeros((400,400))
    MAP_2 = np.zeros((40,40))
    MAP_1_list = []
    MAP_2_list = []
    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    
    # -- ros function
    rospy.Subscriber(name='/solamr_1/scan_lidar', data_class=LaserScan, callback=cb_scan)

    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=2)
    while not rospy.is_shutdown():
        viewer.update()
        rate.sleep()
