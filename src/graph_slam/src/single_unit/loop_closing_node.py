#!/usr/bin/env python
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from car_msg.msg import Node, Edge, Optimized_Node
from std_msgs.msg import Bool

# from scipy import sparse
# from scipy.sparse.linalg import inv, spsolve
# from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
# import csv
# import tf
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import JointState # For subscrbing JointState to compute odom
# from sensor_msgs.msg import LaserScan # For subscribing Laser Scan
# from sklearn.neighbors import NearestNeighbors # For ICP


# -------------- CLASS --------------


class DATA_COLLECTOR():
    
    def __init__(self, topic_NODE, topic_EDGE):

        self.node_set = []
        self.edge_set = []


        # Publisher Declaration
        # self.optimized_node_pub = rospy.Publisher("/solamr_1/optimized_node", Optimized_Node, queue_size=10)

        # Subscriber Declaration
        rospy.Subscriber(topic_NODE, Node, self._get_node)
        rospy.Subscriber(topic_EDGE, Edge, self._get_edge)
        

    def _get_node(self, node_data):

        self.node_set.append([
            node_data.Node_ID,
            node_data.global_pose.x,
            node_data.global_pose.y,
            node_data.global_pose.yaw,
            node_data.Landmark,
            node_data.Scan
        ])

    def _get_edge(self, edge_data):
        
        cov = np.reshape(np.array(edge_data.covariance), (edge_data.covariance_shape.row, edge_data.covariance_shape.row))
        
        self.edge_set.append([
            edge_data.Node_ID_From,
            edge_data.Node_ID_To,
            [   
                edge_data.relative_pose.x,
                edge_data.relative_pose.y,
                edge_data.relative_pose.yaw
            ],
            cov
        ])
    
# -------------- GLOBAL FUNCTION --------------

# -------------- MAIN --------------
if __name__ == "__main__":
    rospy.init_node('Loop_Closing_NODE', anonymous=True)
    rate = rospy.Rate(10)

    try:
        
        info = DATA_COLLECTOR(  topic_NODE = '/solamr_1/collector_node',
                                topic_EDGE = '/solamr_1/collector_edge', 
                                )
        
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        pass
        
        