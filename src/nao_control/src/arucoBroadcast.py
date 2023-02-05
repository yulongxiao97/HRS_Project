#!/usr/bin/env python

from re import T
import rospy
import sys
import cv2
from std_msgs.msg import String
import motion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 
import time
# from pathlib import Path
import matplotlib.pyplot as plt
import tf
from nao_control.srv import ArucoDetector
from geometry_msgs.msg import Twist

# TNOTES: 
# this is the frame of which this code establish. 
# rosrun tf tf_echo /TOPCAMERAFRAME /ARUCOFRAME
# rosrun tf tf_echo /CameraBottom_frame /ARUCOFRAME
# CameraBottom_frame

# Set the map frame, ie, world frame to a reference frame of robot 
# static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
# run like this: 
# rosrun tf static_transform_publisher 0.0 0 0.0 0.0 0.0 0.0 map CameraTop_frame 180

# run rviz GUI
# rosrun rviz rviz

class arucodetection:
    # adapted from https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
    # pose estimation adapted from https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/pose_estimation.py

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",
                                        Image,self.cameraCallback)
        self.pub = rospy.Publisher('aruco_coordinate', String, queue_size=10)
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.camera_params = []
        self.camera_matrix = np.asarray([[551.543059,0.000000,327.382898], [0.000000, 553.736023,225.026380 ], [0.000000,  0.000000, 1.000000]])
        self.distortion_coefficients = np.asarray([-0.066494,0.095481,-0.000279,0.002292,0.000000])

        self.cx = 0
        self.cy = 0
        self.tvec = 0
        self.rvec = 0
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)

        self.troubleshoot = True
        self.ltrans = 0
        self.lrot = 0

        self.end = False
        self.detected = False

        rospy.Timer(rospy.Duration(1), self.timerCallback)
        self.TIMER = 0


    def timerCallback (self, event):
        # print("[ARUCO] " + str(self.TIMER) + " seconds")
        self.TIMER= self.TIMER+1

    def cameraCallback (self, image):
        try: # if there is an image
        
        # Acquire the image, and convert to single channel gray image
            raw_img = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(raw_img, self.arucoDict,
                parameters=arucoParams)

            if self.troubleshoot:
                cv2.imshow("Image from NAO", raw_img)
            # print('Aruco Corners {},aruco IDS {}'.format(corners,ids))

            image_aruco = raw_img.copy()
            
            if len(corners) > 0:
                self.TIMER = 0
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # Draw the corners and ids
                image_aruco = self.markerboundary(image_aruco, corners, ids)

                # also draw Detected marker but doesn't give id 
                # cv2.aruco.drawDetectedMarkers(image_aruco, corners) 

                # Call this function to take in the coordinate frame on top of the Aruco
                image_aruco = self.pose_estimation(image_aruco,corners,ids, self.camera_matrix, self.distortion_coefficients)
                # print(type(self.tvec))
                # print((self.tvec.shape))
                roll, pitch, yaw = self.rvec.squeeze()
                self.tfbroadcaster.sendTransform(
                    translation=self.tvec.squeeze(), 
                    rotation= tf.transformations.quaternion_from_euler(roll, pitch, yaw, axes="sxyz"), 
                    time = rospy.get_rostime(),
                    child = 'ARUCOFRAME',
                    # parent = 'TOPCAMERAFRAME')
                    parent= 'CameraTop_optical_frame')
                    # parent= 'torso')
                    #parent= 'CameraTop_frame')
                self.detected = True
            else: 
                if self.TIMER > 5:
                    self.detected= False
                    self.end= True
                    self.TIMER= 0
                

            if self.troubleshoot:
                cv2.imshow("3D marker position", image_aruco)
            string_to_publish = '{} {}'.format(self.cx, self.cy)
            # self.pub.publish(string_to_publish)
            # print(self.lrot)
            # print(self.ltrans)
            # self rvec is the rotation angle, and tvec is the translation 

        except CvBridgeError as e:
            print (e)

        cv2.waitKey(5)             

    def markerboundary(self,image_aruco, corners, ids):
        for (markerCorner, markerID) in zip(corners, ids):
            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image_aruco, topLeft, topRight, (255, 0, 0), 2)
            cv2.line(image_aruco, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image_aruco, bottomRight, bottomLeft, (0, 0, 255), 2)
            cv2.line(image_aruco, bottomLeft, topLeft, (0, 255, 0), 2)

            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image_aruco, (cX, cY), 4, (0, 0, 255), -1)
            # draw the ArUco marker ID on the image
            # cv2.putText(image_aruco, 'id = {}'.format(markerID),
            #     (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
            #     1, (0, 255, 0), 2)
            self.cx = cX
            self.cy = cY
        return image_aruco
         

    def pose_estimation(self, frame, corners,ids, camera_matrix, distortion_coefficients):
        # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
        for i in range(len(ids)):
            self.rvec, self.tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.03, camera_matrix,
                                                                        distortion_coefficients)


            # Draw Axis
            cv2.aruco.drawAxis(frame, camera_matrix, distortion_coefficients, self.rvec, self.tvec, 0.02)  

        return frame

    def getArucoFromTorso(self):
        '''Re-establish position of the sticks relative to NAO's toso frame'''
        time.sleep(1)
        if self.detected == True:
            try:
                (ltrans,lrot) = self.tflistener.lookupTransform("/torso","/ARUCOFRAME",rospy.Time(0))

                self.ltrans = ltrans
                self.lrot = lrot 
                self.end = True
            except:
                pass
    
    def check(self):
        try:
            self.tfbroadcaster.sendTransform(
            translation=self.ltrans, 
            rotation= self.lrot, 
            time = rospy.get_rostime(),
            child = 'Check',
            # parent = 'TOPCAMERAFRAME')
            parent= 'torso')
            # parent= 'torso')
            #parent= 'CameraTop_frame')
        except:
            pass


def handle_broadcast(req):
    ic = arucodetection()
    rate = rospy.Rate(5)

    while not ic.end:
        print("[WHILE]: " + str(ic.end))
        ic.getArucoFromTorso()
        rate.sleep()

    # Sends the response to the client
    print("[WHILE]: OUT OF THE WHILE " + str(ic.end))
    if ic.detected:
        new_pos = Twist()
        new_pos.linear.x = ic.ltrans[0]
        new_pos.linear.y = ic.ltrans[1]
        new_pos.linear.z = ic.ltrans[2]
        new_pos.angular.x = ic.lrot[0]
        new_pos.angular.y = ic.lrot[1]
        new_pos.angular.z = ic.lrot[2]
    else:
        new_pos = Twist()
        new_pos.linear.x = 0
        new_pos.linear.y = 0
        new_pos.linear.z = 0
        new_pos.angular.x = 0
        new_pos.angular.y = 0
        new_pos.angular.z = 0

    return new_pos


if __name__ == '__main__':
	
    rospy.init_node('ArucoBroadcast', anonymous=True)
    rospy.Service('arucobroadcast', ArucoDetector, handle_broadcast)
    rospy.spin()
