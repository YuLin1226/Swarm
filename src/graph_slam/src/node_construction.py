#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import numpy as np
import tf


class NODE():
    def __init__(self):
        
        # ROS Setup
        rospy.init_node('node_data_extraction', anonymous=True)
        # self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        

        
        self.node_set = []
        self.node_id = -1
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.rate = rospy.Rate(1)
        

        # Main Loop
        rospy.Subscriber('/solamr_1/odom', Odometry, self.get_odom)
        

        while not rospy.is_shutdown():

            self.node_set.append([
                self.node_id,
                self.x,
                self.y,
                self.yaw
            ])

            print("============================")
            print("id: %i"  %self.node_set[-1][0])
            print("x: %f"   %self.node_set[-1][1])
            print("y: %f"   %self.node_set[-1][2])
            print("yaw: %f" %self.node_set[-1][3]) 
        

            self.node_id += 1
            self.rate.sleep()

            
        rospy.spin()
        

    def get_odom(self, odom_msg):
        
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        quaternion= (  odom_msg.pose.pose.orientation.x,
                            odom_msg.pose.pose.orientation.y,
                            odom_msg.pose.pose.orientation.z,
                            odom_msg.pose.pose.orientation.w )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]
        
        

if __name__ == '__main__':
    try:
        a = NODE()

    except KeyboardInterrupt:
        pass

    finally:
        pass