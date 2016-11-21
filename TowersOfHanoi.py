#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import roslib
roslib.load_manifest('obi_package')
import sys
import rospy
import cv2
import time 
import glob 
from baxter_core_msgs.msg import DigitalIOState, EndpointState
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, Range 
from cv_bridge import CvBridge, CvBridgeError
import os.path
import baxter_interface
import errno
import argparse
import time
import struct
import math
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class gripperCameraFeed(object):

    def __init__(self, side='left'):
        self.side = side
        self.cameraPath = "/cameras/%s_hand_camera/image" % (self.side) 
        #which gripper camera is being viewed
        self.imageSub = rospy.Subscriber(self.cameraPath, Image,self.colorFilter)
        #subscribe to receive data from camera
        self.bridge = CvBridge()
        self.cXpixel, self.cYpixel = None, None 
    
    def colorFilter(self, data):
        minArea = 3000
        boundaries = [([110, 50, 50], [130, 255,255])] #blue filter 
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e: print(e)

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper) #mask for color
        output = cv2.bitwise_and(hsv, hsv, mask = mask) #apply mask to img
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE) #find the contours

        for i, c in enumerate(contours):
            area = cv2.contourArea(c)
            M = cv2.moments(c)

            if(area > minArea):
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])

                self.cXpixel, self.cYpixel = cX, cY

                cv2.drawContours(img,contours,i,(0,0,255),3)#draw contour and circle on center of image 
                cv2.circle(img, (cX, cY), 20, (0, 0, 0), 2) # draw black circle
                center_circle = cv2.circle(img, (cX, cY), 20, (0, 0, 0), 1) #center circle on image 
                cv2.putText(img, "Center", (cX - 20, cY - 20), #put text on circle 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
                #optimal pixel coordinate: 382, 13(1-4) in 640x400 image  
        cv2.imshow("Color Detection", np.hstack([img, output]))
        cv2.waitKey(1)

class Locomotion(gripperCameraFeed):
    """
    important note: The different towers are placed 28 cm apart, 
    approx one 11 x 8.5 sheet of paper for ease of resetting environment
    """
    defaultPositions = None

    def __init__(self):
        gripperCameraFeed.__init__(self) #grab init data from parent class
        self.cubeHeight = 40 #mm 
        self.gripper = baxter_interface.Gripper(self.side)
        self.limb = baxter_interface.Limb(self.side)

        circleButtonPath = "/robot/digital_io/%s_lower_button/state" % self.side
        ovalButtonPath = "/robot/digital_io/%s_upper_button/state" % self.side
        irRangePath = "/robot/range/%s_hand_range/state" % self.side

        self.irSensor = rospy.Subscriber(irRangePath, Range, self.irRangeSensor,
                                                        callback_args=self.side)
        self.circleButton = rospy.Subscriber(circleButtonPath, DigitalIOState, 
                            self.openGripper, callback_args=self.side)
        self.ovalButton = rospy.Subscriber(ovalButtonPath, DigitalIOState, 
                                self.closeGripper, callback_args = self.side)

        self.gripperOpen = False 
        self.irRange = None 
        self.armDefaultPosition = None 
        #set to true to overwrite default arm position 
        self.resetDefaultPositions = True

    def grabCenter(self):
        #get the center x and y coordinates
        if self.cXpixel != None or self.cYpixel != None:
            print(self.cXpixel, self.cYpixel)
            return(self.cXpixel, self.cYpixel)
        else:
            time.sleep(1) #avoid stack overflow 
            self.grabCenter()

    def getCurrentArmPosition(self):
        #returns dictionary of current joint angles for the selected arm 
        return self.limb.joint_angles()

    def setArmDefaultPosition(self):
        if self.resetDefaultPositions:
            #reset arm default position
            self.armDefaultPosition =  self.getCurrentArmPosition()
            #gets current arm Position
            file = open(Locomotion.defaultPositionFile, 'w')
            repr(file)
            #delete old positions and write in new ones
            file.close()
        else:
            #read file contents for default arm positions 
            file = open(Locomotion.defaultPositionFile, 'r')
            self.armDefaultPosition = file.read()

        print(self.armDefaultPosition)

        #self.right_limb.move_to_joint_positions(self.armDefaultPosition, 
        #                                timeout=15.0, threshold=0.003726646)

    def calibrateGripper(self):
        #calibrate gripper 
        self.gripper.calibrate(block=True, timeout=2.0)
        self.gripPosition = self.gripper.position()
        self.gripper.command_position(position=0.0, block =True, timeout=5.0)


    def openGripper(self, msg, side): 
        #0 = not pressed, 1 = pressed
        side = self.side
        if(msg.state == 1):
            #opens the gripper
            self.gripper.command_position(position=100.0, block =True,timeout=5.0)
        self.gripperOpen = True 


    def closeGripper(self, msg, side):
        #controlled by oval wrist button
        side = self.side
        if msg.state == 1:
            #closes the gripper
            self.gripper.command_position(position=0.0, block =True,timeout=5.0)
        self.gripperOpen = False

    def irRangeSensor(self, msg, side):
        pass
        #optimal IR Range: .1289 <= range <= .1439
        if self.gripperOpen:
            side = self.side
            #print(msg.range) 


        
    

class solveTowers(object):

    instructions = []
    #stores instructions for solving tower 

    def __init__(self, stack):
       self.stack = stack 
       self.source = 0
       self.target = 1
       self.temp = 2 

    def hanoi(self):
        self.move(self.stack, self.source, self.target, self.temp)

    def move(self, stack, source, target, temp):
        if stack == 1:
            solveTowers.instructions += [(source, target)]
        else:
            self.move(stack-1, source, temp, target)
            self.move(1, source, target, temp)
            self.move(stack-1, temp, target, source)





def main(args):
    rospy.init_node("gripperCameraFeed", anonymous=True)
    cam = gripperCameraFeed()
    solve = solveTowers(3)
    solve.hanoi()
    print(solve.instructions)
    move = Locomotion()
    move.grabCenter()
    move.calibrateGripper()
    move.setArmDefaultPosition()
    
    


    try:    
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)




