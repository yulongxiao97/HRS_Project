#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
# from move_arms.srv import GetPos, MoveJoints

from geometry_msgs.msg import Twist
from std_srvs.srv import *
motionProxy = 0


def handle_standup(req):
    # motionProxy.wbEnabled(True)
    postureProxy.goToPosture("StandInit",0.5)
    return EmptyResponse()
    




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
    # motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('standup_server')

    #init service MoveArms
   
    rospy.Service('StandUp', Empty, handle_standup)

    rospy.spin()
			
		
