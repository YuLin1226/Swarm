#!/usr/bin/env python
import rospy
import math
import geometry_msgs.msg
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
import tf
import numpy as np
import matplotlib.pyplot as plt


class LiDAR_Association():
    def __init__(self, topic_name='/solamr_1/scan_lidar'):
        rospy.init_node('get_laser_scan_data', anonymous=True)



        
        self.fig = plt.figure()
        # self.vel_pub = rospy.Publisher('/solamr_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        # self.cmd = geometry_msgs.msg.Twist()
        rospy.Subscriber(topic_name, LaserScan, self.get_lidar)
        
        

    def get_lidar(self, msg):
        point = []
        data = []
        for ind, dist in enumerate(msg.ranges):
            if dist >= msg.range_min and dist <= msg.range_max:
                
                x = dist * math.cos(ind * math.pi / 725)
                y = dist * math.sin(ind * math.pi / 725)
                point.append([
                    x,
                    y
                ])
                data.append([
                    ind * math.pi / 725,
                    dist
                ])
        self.px = [i[0] for i in point]
        self.py = [i[1] for i in point]
        self.phi = [i[0] for i in data]
        self.dist = [i[1] for i in data]

if __name__ == "__main__":
    try:
        a = LiDAR_Association()
        # b = LiDAR_Association()
        rospy.spin()
    except KeyboardInterrupt:
        pass

    finally:
        # print(b.px)
        plt.scatter(a.phi, a.dist, s=5, color="black")
        # plt.scatter(b.px, b.py, s=5, color="red")
        plt.show()
        pass
