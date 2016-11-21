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
        #cv2.imshow("Color Detection", np.hstack([img, output]))
        #cv2.waitKey(1)


class Locomotion(gripperCameraFeed):
    """
    important note: The different towers are placed 28 cm apart, 
    approx one 11 x 8.5 sheet of paper for ease of resetting environment

    Ideal self.cX, cY values = 305, 138
    """
    defaultPosition = {
                        'left_w0': -3.0418839023767754, 
                        'left_w1': -0.5437961893053792, 
                        'left_w2': -1.3648594060210468, 
                        'left_e0': -0.03796602450016399, 
                        'left_e1': 2.1586944637517487, 
                        'left_s0': -0.40803888957752005, 
                        'left_s1': -1.1593059804444015
                        }
    cubeHeight = .04 #meters
    towerSpacing = .28 #meters
    blockGrabHeight = -0.16569538105206535 
    #z required to pick up single block resting on table, from endpoint_pose()

    def __init__(self):
        gripperCameraFeed.__init__(self) #grab init data from parent class
        
        #paths for subscription
        circleButtonPath = "/robot/digital_io/%s_lower_button/state" % self.side
        ovalButtonPath = "/robot/digital_io/%s_upper_button/state" % self.side
        irRangePath = "/robot/range/%s_hand_range/state" % self.side

        #subscription/api module instances
        self.gripper = baxter_interface.Gripper(self.side)
        self.limb = baxter_interface.Limb(self.side)
        self.irSensor = rospy.Subscriber(irRangePath, Range, self.IRrangeSensor,
                                                        callback_args=self.side)
        self.circleButton = rospy.Subscriber(circleButtonPath, DigitalIOState, 
                            self.openGripper, callback_args=self.side)
        self.ovalButton = rospy.Subscriber(ovalButtonPath, DigitalIOState, 
                                self.closeGripper, callback_args = self.side)

        #administrative instances
        self.maxBlockCenterCalls = 5 
        #if block center not found after n attempts give up 
        self.gripperOpen = True 
        self.IRrange = None #reading from wrist ir sensor 
        #set to true to print out new default arm position, save to class var
        self.resetDefaultPositions = False 
        self.armReady = False #waiting to be in default position 

        self.idealBlockCx, self.idealBlockCy = 305, 138 #pixel goal for gripper alignment 
        self.centerOffsetTolerance = .03 #percent, max center offset tolerance
        cX, cY = self.idealBlockCx,self.idealBlockCy
        delta = self.centerOffsetTolerance

        self.blockCxRange = list(range(int(cX - cX*delta), int(cX + cX*delta)))
        self.blockCyRange = list(range(int(cY - cY*delta), int(cY + cY*delta)))

    def getBlockCenter(self, attempts=0):
        #get the center x and y coordinates
        if attempts == self.maxBlockCenterCalls:
            #give up after 5 attempts to locate center 
            print('Cannot find block center. Giving up')
            return False 
        if self.cXpixel != None or self.cYpixel != None:
            return(self.cXpixel, self.cYpixel)
        else:
            time.sleep(.5) #pause method to avoid stack overflow 
            self.getBlockCenter(attempts+1)

    def getCurrentArmPosition(self):
        #returns dictionary of current joint angles for the selected arm 
        return self.limb.joint_angles()

    def setArmDefaultPosition(self):
        if self.resetDefaultPositions:
            #reset arm default position
            print(self.getCurrentArmPosition())
        else:
           #set arm to default position 
           self.limb.move_to_joint_positions(Locomotion.defaultPosition, 
                                        timeout=15.0, threshold=0.003726646)

    def calibrateGripper(self):
        #calibrate gripper and close it 
        self.gripper.calibrate(block=True, timeout=2.0)
        self.gripper.command_position(position=100.0, block =True, timeout=5.0)


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

    def IRrangeSensor(self, msg, side):
        side = self.side
        self.IRrange = msg.range


    def readInstuctions(self):
        #loops through instructions, 
        # optimal z = -0.16569538105206535
        for move in solveTowers.instructions:
            
            grabStack, grabStackHeight, placeStack, placeStackHeight = move

            blockCx, blockCy = self.getBlockCenter() #get center of block 
            pose = self.limb.endpoint_pose() #gripper pos (x, y, z) + Quaternion 
            print(pose)
            if blockCx in self.blockCxRange and blockCy in self.blockCyRange:
                pose = self.getReachDepth(grabStackHeight, pose)# BROKEN*****
                #check IR range sensor 
                #call place Block
                #move to next stack 
                print('Ready for grab')

            else:
                #use IK solver for adjustment as opposed to just going to next stack
                print('Adjust')
            
            #get the block Center 
            #calls IkSolver to move arm
            #calls IRreader
            #calls grabBlock
            #calls placeBlock  

    def getReachDepth(self, stackHeight, pose):
        #gets reach depth for varying blockHeights 
        cubeHeight = Locomotion.cubeHeight #grab cube size
        grabHeight = Locomotion.blockGrabHeight #grab single block reach depth 
        reachDepth = grabHeight + stackHeight*cubeHeight 
        offset = abs(pose['position'].z - reachDepth) 
        """
        CAN'T SET ATTRIBUTE 
        #del pose['position'].z
        #pose['position'].z = reachDepth

        for position in pose:
            pose['position'].z -= offset

        print(pose['position'].z)
        #new z endpoint for IK solver to use 
        return pose
        """
        pass 

    def IkSolver(self):
        pass

    def grabBlock(self):
        #takes in instructions from IK solver 
        pass

    def IRreader(self):
        pass

    def placeBlock(self):
        pass 


class solveTowers(object):

    instructions = []
    #stores instructions for solving tower 
    #format = [(grabFrom, grabStackHeight, placeHere, placeStackHeight)]

    def __init__(self, stack):
       self.stack = stack 
       self.source = 0
       self.target = 1
       self.temp = 2 

       #changeStackHeightsMethod will use vals to calc. stack heights 
       self.sourceStack = stack
       self.targetStack = 0 
       self.tempStack = 0 

    def changeStackHeights(self, grabFrom, placeHere):
        #manipulate stack heights for 'instructions' class attribute
        if grabFrom == self.source:
            self.sourceStack -= 1  
            if placeHere == self.target: 
                self.targetStack += 1
                return (self.sourceStack + 1, self.targetStack - 1)
            self.tempStack += 1 
            return (self.sourceStack + 1, self.tempStack - 1)
        elif grabFrom == self.temp:
            self.tempStack -= 1
            if placeHere == self.target: 
                self.targetStack += 1
                return (self.tempStack + 1, self.targetStack -1) 
            self.sourceStack += 1 
            return (self.tempStack + 1, self.sourceStack -1)
        else:
            self.targetStack -= 1
            if placeHere == self.source: 
                self.sourceStack += 1
                return (self.targetStack + 1, self.sourceStack -1)
            self.tempStack += 1 
            return (self.targetStack + 1, self.tempStack - 1)

    def hanoi(self):
        #wrapper for solver 
        self.move(self.stack, self.source, self.target, self.temp)

    def move(self, stack, source, target, temp):
        
        if stack == 1:
            sourceHeight, targetHeight = self.changeStackHeights(source, target)
            #get stackHeights
            solveTowers.instructions += [(source, sourceHeight,
                                             target, targetHeight)]
            #add to growing instructions list 
        else:
            self.move(stack-1, source, temp, target)
            self.move(1, source, target, temp)
            self.move(stack-1, temp, target, source)




def main(args):
    rospy.init_node("gripperCameraFeed", anonymous=True)
    cam = gripperCameraFeed() 
    solve = solveTowers(1)
    solve.hanoi()
    move = Locomotion()
    move.setArmDefaultPosition()
    move.calibrateGripper()
    move.readInstuctions()

    
    


    try:    
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



