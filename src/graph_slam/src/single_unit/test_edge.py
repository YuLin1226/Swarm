#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
from scipy.sparse.linalg import inv, spsolve
from car_msg.msg import Node, Edge, Optimized_Node, Node_List, Edge_List
from std_msgs.msg import Bool



class DATA_COLLECTOR():
    
    def __init__(self, topic_NODE, topic_EDGE):

        self.fig, self.ax = plt.subplots(2,1)
        plt.ion()
        plt.show()

        self.node_set = []
        self.edge_set = []

        self.node_px = None
        self.node_py = None
        self.edge_px = None
        self.edge_px = None

        # Subscriber Declaration
        rospy.Subscriber(topic_NODE, Node_List, self._get_node)
        rospy.Subscriber(topic_EDGE, Edge_List, self._get_edge)

    def _get_node(self, node_data):
        self.node_set = []
        for i in range(node_data.Number_Node):

            self.node_set.append([
                node_data.Node_ID[i],
                node_data.global_pose_x[i],
                node_data.global_pose_y[i],
                node_data.global_pose_yaw[i]
            ])

        self.node_px = np.array([p[1] for p in self.node_set])
        self.node_py = np.array([p[2] for p in self.node_set])


    def _get_edge(self, edge_data):
        
        # cov = np.reshape(np.array(edge_data.covariance), (edge_data.covariance_shape.row, edge_data.covariance_shape.row))
        cov = np.array([[20,0,0],[0,20,0],[0,0,1000]])
        self.edge_set = []
        for i in range(edge_data.Number_Edge):
            self.edge_set.append([
                edge_data.Node_ID_From[i],
                edge_data.Node_ID_To[i],
                [   
                    edge_data.relative_pose_x[i],
                    edge_data.relative_pose_y[i],
                    edge_data.relative_pose_yaw[i]
                ],
                cov
            ])

    def update(self):
        """
        Update matplotlib ax.
        """
        self._plot_data()
        plt.pause(0.01)
        self.fig.canvas.draw()

    
    def _plot_data(self):
        
        self.ax[0].cla() 
        self.ax[0].plot(self.node_px, self.node_py)
        self.ax[0].grid(which='both', color='grey')

        self.ax[1].cla() 
        self.ax[1].plot(self.edge_px, self.edge_py)
        self.ax[1].grid(which='both', color='grey')

# -------------- MAIN --------------
if __name__ == "__main__":
    rospy.init_node('Graph_Optimization_NODE', anonymous=True)
    rate = rospy.Rate(10)

    try:
        
        info = DATA_COLLECTOR(  topic_NODE = '/solamr_1/collector_node',
                                topic_EDGE = '/solamr_1/collector_edge')
        
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
        