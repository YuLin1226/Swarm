#!/usr/bin/env python

from __future__ import print_function
from pso_communication.srv import getBest, getBestResponse
import rospy

def callback(req):
    print("Personal Best Value = %s"%req.value)
    print("Position : (%s,%s)"%(req.position_x,req.position_y))
    return getBestResponse(req.value + 1, req.position_x, req.position_y)

def server():
    rospy.init_node('creating_server_for_communications')
    s = rospy.Service('pso_communication', getBest, callback)
    print("Ready to communicate.")
    rospy.spin()

if __name__ == "__main__":
    server()