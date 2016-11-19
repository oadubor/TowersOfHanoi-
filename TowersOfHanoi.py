
#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import roslib
roslib.load_manifest('obi_package')
import sys
import rospy
import cv2
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

class cameraFeed(object):

	def __init__(self, side='right'):
		cameraPath = "/cameras/%s_hand_camera/image" % (side) 
		#which gripper camera is being viewed
		self.imageSub = rospy.Subscriber(cameraPath, Image, self.getVideoFrames)
		#subscribe to receive data from camera
		self.bridge = CvBridge()
		#self.cvImage = self.bridge.imgmsg_to_cv2(data, 'bgr8')

	def getVideoFrames(self, data):
		try:
			cvImage = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)
		
		discardFrames = 30
	 	#toss these frames for camera to adjust to light levels in room 
		# Now we can initialize the camera capture object with the cv2.VideoCapture class.
		# All it needs is the index to a camera port.
		#camera = cv2.VideoCapture(camera_port)
		 
		# Captures a single image from the camera and returns it in PIL format
		def getImage():
		 # read is the easiest way to get a full image out of a VideoCapture object.
		 	ret, image = cvImage.read()
			return image
		 
		# Ramp the camera - these frames will be discarded and are only used to allow v4l2
		# to adjust light levels, if necessary
		for i in xrange(discardFrames):
			temp = getImage()
		print("Taking image...")
		# Take the actual image we want to keep
		cameraCapture = get_image()
		file = "/home/fury_ws/bax_local/src/obi_package/images/testImageNov18.png"
		# A nice feature of the imwrite method is that it will automatically choose the
		# correct format based on the file extension you provide. Convenient!
		cv2.imwrite(file, cameraCapture)
		
		# You'll want to release the camera, otherwise you won't be able to create a new
		# capture object until your script exits
		
		
		#cv2.imshow("Camera Feed", cvImage)
		#cv2.waitKey(1)


def main(args):
	rospy.init_node("cameraFeed", anonymous=True)
	cam = cameraFeed()

	try:	
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)



