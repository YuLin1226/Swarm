#!/usr/bin/env python

from __future__ import print_function
from pso_communication.srv import getBest
import sys
import rospy

def add_two_ints_client(value, x, y):
    rospy.wait_for_service('pso_communication')
    try:
        resp = rospy.ServiceProxy('pso_communication', getBest)
        resp_data = resp(value, x, y)
        print("Global Best Value = %s"%resp_data.value)
        print("Position : (%s,%s)"%(resp_data.position_x,resp_data.position_y))
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    if len(sys.argv) == 4:
        value = float(sys.argv[1])
        x = float(sys.argv[2])
        y = float(sys.argv[3])
    else:
        sys.exit(1)
    add_two_ints_client(value, x, y)