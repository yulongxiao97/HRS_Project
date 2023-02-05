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


def handle_MoveArmsDown(req):

    arm_joint= ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw","LHand", 
                "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]

    stand_pose = [0.0030260086059570312, 0.009161949157714844, 
                1.440384030342102, 0.269942045211792, -1.3683700561523438, -0.98785400390625, -0.013848066329956055, 0.258400022983551, 
                -0.004559993743896484, 4.1961669921875e-05, -0.4494199752807617, 0.7009961605072021, -0.3497939109802246, 4.1961669921875e-05, 
                -0.004559993743896484, 0.0015759468078613281, -0.4617760181427002, 0.7010800838470459, -0.34970998764038086, 4.1961669921875e-05, 
                1.4496722221374512, -0.2715599536895752, 1.4005000591278076, 0.9879379272460938, -0.0123138427734375, 0.25599998235702515]


    # angles_grasp =  [-0.00, -0.15660005569458008, 
    #                 0.15949392318725586, -0.22921796417236328, -0.7624399662017822, -0.6687819957733154, -0.8222661018371582, 0.8519999980926514, 
    #                 0.0583338737487793, 0.042994022369384766, -0.9203581809997559, 2.1076741218566895, -1.185823917388916, -0.024502038955688477, 
    #                 0.0583338737487793, -0.024502038955688477, -0.9066357612609863, 2.103156089782715, -1.186300277709961, 0.024585962295532227, 
    #                 0.22093796730041504, 0.229161949157714844, 0.6534421443939209, 0.7731781005859375, 0.9249601364135742, 0.974399983882904]


    # angles_grasp =  [0.020949392318725586, -0.22921796417236328, -0.7624399662017822, -0.6687819957733154, -0.8222661018371582, 0.8519999980926514, 
    #                 0.02093796730041504, 0.229161949157714844, 0.6534421443939209, 0.7731781005859375, 0.9249601364135742, 0.974399983882904]

    #motionProxy.setWalkArmsEnabled(False, False)
    arm_grasp = [1.3560140132904053, -0.233338737487793, -1.0048117637634277, -1.0737581253051758, -0.257753849029541, 0.8467999696731567, 
                1.3560140132904053, 0.2333404083251953, 1.0019501209259033, 1.055723762512207, 0.25368995666503906, 0.8479999876022339]
    
          
    time.sleep(2.0)
    fractionMaxSpeed  = 0.03
    # motionProxy.setAngles(arm_joint, angles_grasp, fractionMaxSpeed)
    # time.sleep(2.0)
    
    motionProxy.setAngles(arm_joint, arm_grasp, fractionMaxSpeed)
    motionProxy.waitUntilMoveIsFinished()
    time.sleep(2.0)
    return EmptyResponse()
    




if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_arms_down_server')

    #init service MoveArms
   
    rospy.Service('MoveArmsDown', Empty, handle_MoveArmsDown)

    rospy.spin()
			
		
