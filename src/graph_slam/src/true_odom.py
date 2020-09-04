
#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

def odom_cb(data):
    x.append(data.pose.pose.position.x)
    y.append(data.pose.pose.position.y)


if __name__ == '__main__':
    
    rospy.init_node('path_node')
    odom_sub = rospy.Subscriber('/solamr_1/odom', Odometry, odom_cb)

    x = []
    y = []

    try:
        rospy.spin()

    except KeyboardInterrupt:
        pass

    finally:
        plt.figure()
        plt.xlim((-10, 10))
        plt.ylim((-10, 10))
        plt.plot(x, y)
        plt.scatter(x, y, s=2)
        plt.show()
        