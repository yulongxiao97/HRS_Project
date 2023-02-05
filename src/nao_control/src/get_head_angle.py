#!/usr/bin/env python
import rospy
import time
import motion
import numpy as np
import almath
import sys
import cv2
from naoqi import ALProxy
from nao_control.srv import GetPos

from geometry_msgs.msg import Twist
motionProxy = 0

def handle_GetPos(req):
    name = req.joint_name
    frame = 0 #FRAME_TORSO
    useSensorValues = True
    curPos = motionProxy.getPosition(name,frame,useSensorValues)
    cur_pos = Twist()
    cur_pos.linear.x = curPos[0]
    cur_pos.linear.y = curPos[1]
    cur_pos.linear.z = curPos[2]
    cur_pos.angular.x = curPos[3]
    cur_pos.angular.y = curPos[4]
    cur_pos.angular.z = curPos[5]
    return cur_pos

def handle_SetPos(req):
    effector = req.effector
    frame = 0 # FRAME_TORSO
    newPos = [0.0,0.0,0.0,0.0,0.0,0.0]
    newPos[0] = req.new_pos.linear.x
    newPos[1] = req.new_pos.linear.y
    newPos[2] = req.new_pos.linear.z
    newPos[3] = req.new_pos.angular.x
    newPos[4] = req.new_pos.angular.y
    newPos[5] = req.new_pos.angular.z
    speed = req.velocity
    time = req.time
    mask = req.mask
    if speed == 0.0:
        # motionProxy.setStiffnesses('Body',1)
        motionProxy.positionInterpolations(effector,frame, newPos,mask, time)
    else:
        # motionProxy.setStiffnesses('Body',1)
        motionProxy.setPositions(effector, frame, newPos , speed, mask)
    return 1


def handle_tracking(req):
    frame = 0 # FRAME_TORSO
    # original pose
    orig_posL = [0.067,0.197,-0.071,-2.302,0.798,0.796]
    orig_posR = [0.062,-0.145,-0.103,2.157,1.151,-0.566]

    # motionProxy.setStiffnesses('Body',1)

    #if no marker detected, set to original pose
    if req.new_pos.linear.z == 0:
        motionProxy.setPosition("LArm",0,orig_posL, 1.0, 63)
        motionProxy.setPosition("RArm",0,orig_posR, 1.0, 63)
    else: #if marker detected
        name = 'CameraBottom'
        useSensorValues = True
        result = motionProxy.getTransform(name, frame, useSensorValues)

        #homogeneous transform relating CameraBottom_optical to CameraBottom frame
        TF_CamOpt2CamBot = np.array([[0,0,1,0],
                                    [1,0,0,0],
                                    [0,1,0,0],
                                    [0,0,0,1]])
        #rotation matrix relating CameraBottom_optical to CameraBottom frame
        R_CamOpt2CamBot = np.array([[0,0,1],
                                    [1,0,0],
                                    [0,1,0]])
        #homo transform relating CameraBottom frame to Torso
        TF_CamBot2Torso = np.array([[result[0],result[1],result[2],result[3]],
                                    [result[4],result[5],result[6],result[7]],
                                    [result[8],result[9],result[10],result[11]],
                                    [result[12],result[13],result[14],result[15]]])
        #rotation matrix relating CameraBottom frame to Torso
        R_CamBot2Torso = np.array([[result[0],result[1],result[2]],
                                   [result[4],result[5],result[6]],
                                   [result[8],result[9],result[10]]])

        p_new_CamOpt = np.array([req.new_pos.linear.x, req.new_pos.linear.y,req.new_pos.linear.z,1]).reshape(-1,1)
        r_new_CamOpt = np.array([req.new_pos.angular.x, req.new_pos.angular.y,req.new_pos.angular.z]).reshape(-1,1)

        # transform marker position into Torso frame
        p_new_Torso = TF_CamBot2Torso.dot(TF_CamOpt2CamBot.dot(p_new_CamOpt))
        # transform marker orientation into Torso frame
        r_new_CamOpt_33,_ = cv2.Rodrigues(r_new_CamOpt)
        r_new_Torso_33 = R_CamBot2Torso.dot(R_CamOpt2CamBot.dot(r_new_CamOpt_33))
        r_new_Torso,_ = cv2.Rodrigues(r_new_Torso_33)

        new_Torso =[0,0,0,0,0,0]
        new_Torso[0] = p_new_Torso[0][0]
        new_Torso[1] = p_new_Torso[1][0]
        new_Torso[2] = p_new_Torso[2][0]
        new_Torso[3] = r_new_Torso[0][0]
        new_Torso[4] = r_new_Torso[1][0]
        new_Torso[5] = r_new_Torso[2][0]

        #decide which arm will be used
        if new_Torso[1] < -0.02:
            # motionProxy.setStiffnesses('Body',1)
            motionProxy.setPositions("RArm",frame, orig_posR ,1, 63)
            motionProxy.setPositions("LArm",frame, new_Torso ,1, 63)
        elif new_Torso[1] > 0.02:
            # motionProxy.setStiffnesses('Body',1)
            motionProxy.setPositions("LArm",frame, orig_posL ,1, 63)
            motionProxy.setPositions("RArm",frame, new_Torso ,1, 63)
        else:
            # motionProxy.setStiffnesses('Body',1)
            motionProxy.setPositions("LArm",frame, new_Torso ,1, 63)
            motionProxy.setPositions("RArm",frame, new_Torso ,1, 63)

    return 1
    

if __name__ == '__main__':
    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    motionProxy = ALProxy("ALMotion", robotIP, PORT)
    rospy.init_node('move_joints_server')

    #init service GetPos
    rospy.Service('getposition', GetPos, handle_GetPos)

    rospy.spin()
			
		
