#!/usr/bin/env python
import rospy
import numpy as np
import math
import csv
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan




class LiDAR_Association():

    def __init__(self, topic_name='/solamr_1/scan_lidar'):
        
        # self.vel_pub = rospy.Publisher('/solamr_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        # self.px = []
        # self.py = []
        self.scan = np.zeros([])
        rospy.Subscriber(topic_name, LaserScan, self.get_lidar)
        self.po = []


    def get_lidar(self, msg):
        point = []
        self.po.append(msg.ranges)
        for ind, dist in enumerate(msg.ranges):
            if dist >= msg.range_min and dist <= msg.range_max:
                
                x = dist * math.cos(ind * math.pi / 725)
                y = dist * math.sin(ind * math.pi / 725)
                point.append([
                    x,
                    y
                ])
        # self.px = [i[0] for i in point]
        # self.py = [i[1] for i in point]
        self.scan = np.array(point)
        # print(np.array(point).shape)


if __name__ == "__main__":

    rospy.init_node('node_edge_construction', anonymous=True)

    try:
        car1 = LiDAR_Association('/solamr_1/scan_lidar')
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        data = car1.po[-1]
        x = []
        y = []
        for ind, dist in enumerate(data):
            x.append(ind)
            y.append(dist)
        # x = np.array(x)
        # y
        plt.scatter(x,y, s=5, color="red")
        plt.show()
        pass

# # Define 2D List
# data = []
# for i in range(15):
#     data.append([
#         i+1, i+2
#     ])
# for i in range(5):
#     data.append([
#         3*i+1, 3*i+4
#     ])
# print("Init: ", data)
# print("=============")

# # Define start node and end node
# start_node = 1
# end_node = 13
# print("Next Start Node: ", start_node)
# print("End Node: ", end_node)
# print("=============")

# T_list = []

# # Define while loop
# while not start_node == end_node:
#     # Define list x to store desired elements in list random.
#     x = []
#     for ind ,e in enumerate(data):
#         if e[0] == start_node:
#             x.append(data[ind])

#     print("After Selection: ", x)
#     print("=============")

#     # Define list b to store 2nd element in list x. 
#     # This will be used for finding the index of min and max in list x sorted by 2nd elements. 
#     b = [i[1] for i in x]
#     M = b.index(max(b))

#     start_node = x[M][1]
#     T_list.append(x[M])

#     print("Next Start Node: ", start_node)
#     print("End Node: ", end_node)
#     print("=============")

# print("T_new:", T_list)

# T = [
#     np.array([[1],[2],[3]]),np.array([[1],[2],[3]]),np.array([[1],[2],[3]]),np.array([[1],[2],[3]])
# ]

# print(np.sum(T, axis=0))