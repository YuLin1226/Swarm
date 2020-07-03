#!/usr/bin/env python

from __future__ import print_function
from pso_communication.srv import getBest
import sys
import rospy

def add_two_ints_client(value, x, y):
    rospy.wait_for_service('pso_communication')
    try:
        add_two_ints = rospy.ServiceProxy('pso_communication', getBest)
        resp1 = add_two_ints(value, x, y)
        
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