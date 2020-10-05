#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from bresenham import bresenham

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
        plt.pause(0.01)

    def _plot_scan_map_high_resolution(self):
        
        global MAP_1
        self.ax1.cla() 
        self.ax1.imshow(X=MAP_1[::-1], cmap="gray")#, origin="lower")  # use img to show costmap
        self.ax1.grid(which='both', color='grey')
        

    def _plot_scan_map_low_resolution(self):
        
        global MAP_2
        self.ax2.cla() 
        self.ax2.imshow(X=MAP_2[::-1], cmap="gray")#, origin="lower")  # use img to show costmap
        self.ax2.grid(which='both', color='grey')
        

    def _plot_robot(self):
        """
        Plot robot with orientation (assuming the location is at the center).
        """

        global ROBOT_X, ROBOT_Y

        try:
            py = MAP_1.shape[0] - int(100*(ROBOT_X - MAP_1_ORIGIN[0])/5)
            px = int(100*(ROBOT_Y - MAP_1_ORIGIN[1])/5)
            
            self.ax1.plot(px, py, marker='o', markersize=5, linestyle='None')

            py = MAP_2.shape[0] - int(100*(ROBOT_X - MAP_2_ORIGIN[0])/50)
            px = int(100*(ROBOT_Y - MAP_2_ORIGIN[1])/50)
            
            self.ax2.plot(px, py, marker='o', markersize=5, linestyle='None')

        except Exception as err:
            rospy.loginfo(err)

    
# --

def cb_scan(msg):

    global MAP_1, MAP_2, ROBOT_X, ROBOT_Y, ROBOT_YAW, MAP_1_ORIGIN, MAP_2_ORIGIN, LOG_odd_occ, LOG_odd_free

    for ind, dist in enumerate(msg.ranges):
        
        if dist >= msg.range_min and dist <= msg.range_max:
            d = dist   
        elif dist <= msg.range_min:
            d = msg.range_min
        else:
            d = msg.range_max

        x = d * math.cos(ind*math.pi/725 - math.pi/2 + ROBOT_YAW) + ROBOT_X
        y = d * math.sin(ind*math.pi/725 - math.pi/2 + ROBOT_YAW) + ROBOT_Y

        
        # MAP_1
        px = int(100*(x - MAP_1_ORIGIN[0])/5) 
        py = int(100*(y - MAP_1_ORIGIN[1])/5)
        if px < 0:
            new_map = np.zeros((int(500/5), MAP_1.shape[1]))
            MAP_1 = np.vstack((new_map, MAP_1))
            px = px + 100
            MAP_1_ORIGIN[0] = MAP_1_ORIGIN[0] - 5


        elif px > MAP_1.shape[0]:
            new_map = np.zeros((int(500/5), MAP_1.shape[1]))
            MAP_1 = np.vstack((MAP_1, new_map))

        if py < 0:
            new_map = np.zeros((MAP_1.shape[0], int(500/5)))
            MAP_1 = np.hstack((new_map, MAP_1))
            py = py + 100
            MAP_1_ORIGIN[1] = MAP_1_ORIGIN[1] - 5

        elif py > MAP_1.shape[1]:
            new_map = np.zeros((MAP_1.shape[0], int(500/5)))
            MAP_1 = np.hstack((MAP_1, new_map))
        
        # MAP_1[px,py] = -1

        MAP_1[px,py] += LOG_odd_occ
        if MAP_1[px,py] > 1:
            MAP_1[px,py] = 1

        pixel_free = list(bresenham(int(100*(ROBOT_X - MAP_1_ORIGIN[1])/5), int(100*(ROBOT_Y - MAP_1_ORIGIN[1])/5), px, py))
        for i in range(len(pixel_free)-2):
            MAP_1[pixel_free[i+1]] -= LOG_odd_free
            if MAP_1[px,py] < -1:
                MAP_1[px,py] = -1



        # MAP_2
        px = int(100*(x - MAP_2_ORIGIN[0])/50) 
        py = int(100*(y - MAP_2_ORIGIN[1])/50)
        if px < 0:
            new_map = np.zeros((int(500/50), MAP_2.shape[1]))
            MAP_2 = np.vstack((new_map, MAP_2))
            px = px + 10
            MAP_2_ORIGIN[0] = MAP_2_ORIGIN[0] - 5


        elif px > MAP_2.shape[0]:
            new_map = np.zeros((int(500/50), MAP_2.shape[1]))
            MAP_2 = np.vstack((MAP_2, new_map))

        if py < 0:
            new_map = np.zeros((MAP_2.shape[0], int(500/50)))
            MAP_2 = np.hstack((new_map, MAP_2))
            py = py + 10
            MAP_2_ORIGIN[1] = MAP_2_ORIGIN[1] - 5

        elif py > MAP_2.shape[1]:
            new_map = np.zeros((MAP_2.shape[0], int(500/50)))
            MAP_2 = np.hstack((MAP_2, new_map))
        
        # MAP_2[px,py] = -1
        MAP_2[px,py] += LOG_odd_occ
        if MAP_2[px,py] > 1:
            MAP_2[px,py] = 1

        pixel_free = list(bresenham(int(100*(ROBOT_X - MAP_2_ORIGIN[1])/50), int(100*(ROBOT_Y - MAP_2_ORIGIN[1])/50), px, py))
        for i in range(len(pixel_free)-2):
            MAP_2[pixel_free[i+1]] -= LOG_odd_free
            if MAP_2[px,py] < -1:
                MAP_2[px,py] = -1

def cb_odom(data):

    global ROBOT_X, ROBOT_Y, ROBOT_YAW

    _, _, ROBOT_YAW = tf.transformations.euler_from_quaternion([
                                                                data.pose.pose.orientation.x,
                                                                data.pose.pose.orientation.y,
                                                                data.pose.pose.orientation.z,
                                                                data.pose.pose.orientation.w,
    ])

    ROBOT_X = data.pose.pose.position.x
    ROBOT_Y = data.pose.pose.position.y
    



if __name__ == "__main__":

    # -- global vars
    MAP_1 = np.zeros((200,200))
    MAP_2 = np.zeros((40,40))
    MAP_1_ORIGIN = np.zeros((2))
    MAP_2_ORIGIN = np.zeros((2))
    
    ROBOT_X = 0
    ROBOT_Y = 0
    ROBOT_YAW = 0
    
    LOG_odd_occ = 0.9
    LOG_odd_free = 0.7

    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    
    # -- ros function
    rospy.Subscriber(name='/solamr_1/scan_lidar', data_class=LaserScan, callback=cb_scan)
    rospy.Subscriber(name='/solamr_1/wheel_odom', data_class=Odometry, callback=cb_odom)

    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=0.5)
    while not rospy.is_shutdown():
        viewer.update()
        rate.sleep()
