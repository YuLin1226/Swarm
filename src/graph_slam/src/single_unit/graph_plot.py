#!/usr/bin/env python
from matplotlib import pyplot as plt
import numpy as np
import math
import rospy
from car_msg.msg import Node, Edge, Optimized_Node, Node_List, Edge_List
from std_msgs.msg import Bool

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
        self._plot_data()
        plt.pause(0.01)
        self.fig.canvas.draw()
        


    def _plot_data(self):
        
        global X, Y
        self.ax.cla() 
        self.ax.plot(X,Y)
        self.ax.grid(which='both', color='grey')
        


    


def cb_node(msg):

    global X, Y
    X = msg.Optimized_x
    Y = msg.Optimized_y



if __name__ == "__main__":
    
    X, Y = 0, 0

    # -- ros node and params
    rospy.init_node(name="viewer", anonymous=False)
    
    # -- ros function
    graph_pub = rospy.Publisher("/solamr_1/graph_request", Bool, queue_size=10)
    rospy.Subscriber(name="/solamr_1/optimized_node", data_class=Optimized_Node, callback=cb_node)

    # -- matplotlib show 
    viewer = Viewer()
    rate = rospy.Rate(hz=0.1)
    
    cmd = Bool()
    while not rospy.is_shutdown():
        cmd.data = True
        graph_pub.publish(cmd)
        viewer.update()
        rate.sleep()
