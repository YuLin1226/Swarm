#!/usr/bin/env python
import rospy
import math
import geometry_msgs.msg
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
import tf
import numpy as np



class Follower():
    def __init__(self):
        rospy.init_node('reach_vertice', anonymous=True)



        # Define vertice
        self.vertice = [[3,3],[-3,3],[-3,-3],[3,-3]]
        self.vertice.append(self.vertice[0])
        self.flag = 4
        # Define Car Position
        self.car_pose_x = 0
        self.car_pose_y = 0
        self.car_pose_th = 0
        self.car_twist_vx = 0

        # Define Parameters
        self.err_last = 0
        self.v = 0

        # PID
        self.Kp = 0.2
        self.Ki = 1
        self.T = 1/30
    

        self.vel_pub = rospy.Publisher('/solamr_1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        self.cmd = geometry_msgs.msg.Twist()
        rospy.Subscriber('/solamr_1/odom',Odometry, self.get_odom) 
        rospy.spin()


    def get_odom(self, odom_msg):
        self.car_pose_x = odom_msg.pose.pose.position.x
        self.car_pose_y = odom_msg.pose.pose.position.y
        self.car_quaternion= ( odom_msg.pose.pose.orientation.x,
                                odom_msg.pose.pose.orientation.y,
                                odom_msg.pose.pose.orientation.z,
                                odom_msg.pose.pose.orientation.w )
        self.car_euler = tf.transformations.euler_from_quaternion(self.car_quaternion)
        self.car_pose_th = self.car_euler[2]
        self.car_twist_vx = odom_msg.twist.twist.linear.x
        self.calculate_cmd_vel(self.vertice[self.flag][0], self.vertice[self.flag][1])
        
        

    def calculate_cmd_vel(self, px=3, py=3, ref=0.0):
        self.D = math.sqrt( (self.car_pose_x - px)**2 + (self.car_pose_y - py)**2 )
        self.theta = math.atan2((py - self.car_pose_y), (px - self.car_pose_x) ) - self.car_pose_th
        
        if self.theta > math.pi:
            self.theta = self.theta - 2*math.pi
        elif self.theta < -math.pi:
            self.theta = self.theta + 2*math.pi

        self.R = self.D / (2 * math.sin(self.theta))
        self.err = - ref + self.D
        # Switch Target Points
        

        if abs(self.theta) < 30 * 3.1415926 / 180:
            self.v = self.v + self.err*(self.Kp + self.Ki*self.T/2) + self.err_last*(self.Ki*self.T/2 - self.Kp)
            self.w = self.v / self.R
            self.err_last = self.err
            self.cmd.linear.x = self.v
            self.cmd.angular.z = self.w
        else:
            self.cmd.linear.x = 0
            self.cmd.angular.z = np.sign(self.theta)*0.1#(30/180*math.pi)
        
        if self.D < 0.3:
            if self.flag < len(self.vertice) - 1:
                self.flag = self.flag + 1
                self.cmd.linear.x  = 0
                self.cmd.angular.z = 0

        self.vel_pub.publish(self.cmd)
        # print("D: %s"%self.D)
        # print("R: %s"%self.R)
        print("Heading: %s"%(self.car_pose_th/math.pi*180))
        print("Theta: %s"%(self.theta/math.pi*180))
        
        



if __name__ == "__main__":
    try:
        a = Follower()
        while not rospy.is_shutdown():
            pass

    except KeyboardInterrupt:
        pass

    finally:
        pass

