#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from nao_control.srv import NaoWalk

from geometry_msgs.msg import Twist
from std_srvs.srv import *
motionProxy = 0


def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def walk_to(x,y,theta):
    # try:
    #     motionProxy = ALProxy("ALMotion", robotIP, 9559)
    # except Exception as e:
    #     print("[WALK] Could not create proxy to ALMotion")
    #     print("[WALK] Error was: ", e)


    # Set NAO in stiffness On
    # StiffnessOn(motionProxy)

    # Enable arms control by move algorithm
    motionProxy.setWalkArmsEnabled(True, True)
    # motionProxy.setWalkArmsEnabled(False, False)

    # arm_grasp = [0.15949392318725586, -0.22921796417236328, -0.7624399662017822, -0.6687819957733154, -0.8222661018371582, 0.8519999980926514,
    #             0.22093796730041504, 0.229161949157714844, 0.6534421443939209, 0.7731781005859375, 0.9249601364135742, 0.974399983882904]

    # arm_joint= ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw","LHand", 
    #             "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw","RHand"]
    # fractionMaxSpeed  = 0.3
    # motionProxy.angleInterpolationWithSpeed(arm_joint, arm_grasp, fractionMaxSpeed)
    # time.sleep(5.0)

    # FOOT CONTACT PROTECTION
    # motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])


    print("[WALK] Robot Move :", x,y,theta)
    motionProxy.post.moveTo(x, y, theta)
    # wait is useful because with post moveTo is not blocking function
    motionProxy.waitUntilMoveIsFinished()


def handle_NaoWalk(req):
    walk_dist = req.dist
    walk_theta = req.theta
    print(walk_dist,walk_theta)
    if (walk_dist == 0):
        walk_to(0,0,walk_theta)
    else:
        walk_to(0,0,walk_theta)
        walk_to(walk_dist,0,0)
    return True

    

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('nao_walk_server')

    #init service
    rospy.Service('naowalk', NaoWalk, handle_NaoWalk)

    rospy.spin()
			
		
