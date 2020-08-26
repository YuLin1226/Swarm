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


# if __name__ == "__main__":

#     rospy.init_node('node_edge_construction', anonymous=True)

#     try:
#         car1 = LiDAR_Association('/solamr_1/scan_lidar')
#         rospy.spin()

#     except KeyboardInterrupt:
#         pass

#     finally:
#         data = car1.po[-1]
#         x = []
#         y = []
#         for ind, dist in enumerate(data):
#             x.append(ind)
#             y.append(dist)
#         # x = np.array(x)
#         # y
#         plt.scatter(x,y, s=5, color="red")
#         plt.show()
#         pass

Node = [[1,3,4],[2,2,2,2],[3,3,3,3,3,3],[]]
print(len(Node))