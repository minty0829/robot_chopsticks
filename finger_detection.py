#!/usr/bin/env python

import os
import numpy as np
import cv2
import rospy
import math
import matplotlib.pyplot as plt

from sensor_msgs.msg import Range
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
this_file = os.path.dirname(os.path.abspath(__file__))




class FingerDetectionProcess:

	def __init__(self, left_range_sub_topic, right_range_sub_topic, left_image_sub_topic, right_image_sub_topic):
		# self.left_image_sub_topic = rospy.Subscriber(left_image_sub_topic, Image, self.left_finger_callback)
		# self.right_image_sub_topic = rospy.Subscriber(right_image_sub_topic, Image, self.right_finger_callback)
		# self.left_range_sub_topic = rospy.Subscriber(left_range_sub_topic, Range, self.left_range_callback)
		self.right_range_sub_topic = rospy.Subscriber(right_range_sub_topic, Range, self.right_range_callback)
		# self.left_touched_pub_topic = rospy.Publisher()
		self.bridge = CvBridge()
		self.first_image_taken = False
		self.first_image = None
		self.IMG_DIR = '/'.join(this_file.split('/')[:-2]) + '/img'
		self.left_finger_count = 0
		self.right_finger_count = 0
		self.left_finger_count_array = []
		self.right_finger_count_array = []
		

	def farthest_point(self, defects, contour, centroid):
		if defects is not None and centroid is not None:
			s = defects[:, 0][:, 0]
			cx, cy = centroid

			x = np.array(contour[s][:, 0][:, 0], dtype=np.float)
			y = np.array(contour[s][:, 0][:, 1], dtype=np.float)

			xp = cv2.pow(cv2.subtract(x, cx), 2)
			yp = cv2.pow(cv2.subtract(y, cy), 2)
			dist = cv2.sqrt(cv2.add(xp, yp))

			dist_max_i = np.argmax(dist)

			if dist_max_i < len(s):
				farthest_defect = s[dist_max_i]
				farthest_point = tuple(contour[farthest_defect][0])
				return farthest_point
			else:
				return None

	def centroid(self, max_contour):
		moment = cv2.moments(max_contour)
		if moment['m00'] != 0:
			cx = int(moment['m10'] / moment['m00'])
			cy = int(moment['m01'] / moment['m00'])
			return cx, cy
		else:
			return None

	def left_range_callback(self, message):
		# print("LEFT IR:" + str(message.range))
		if message.range <= 0.1:
			print("Left hand has been touched!")

	def right_range_callback(self, message):
		print("RIGHT IR:" + str(message.range))
		if message.range <= 0.2 and message.range > 0.1:
			print("Using human's left hand")
		elif message.range <= 0.1:
			print("Using human's right hand")
	def nothing(self, x):
		pass

	def read_image(self, img_name, grayscale=False):
		if not grayscale:
			img = cv2.imread(img_name)
		else:
			img = cv2.imread(img_name, 0)
		return img	

	def show_image(self, img_name, title='Fig', grayscale=False):
		if not grayscale:
			plt.imshow(img_name)
			plt.title(title)
			plt.show()
		else:
			plt.imshow(img_name, cmap='gray')
			plt.title(title)
			plt.show()


	def left_finger_callback(self, message):
		frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
		cv2.imshow("Left hand:", frame)
		key = cv2.waitKey(30)
		if key == ord("q"):
			cv2.destroyAllWindows()
		return

	def right_finger_callback(self, message):
		frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
		cv2.imshow("Right hand:", frame)
		key = cv2.waitKey(30)
		if key == ord("q"):
			cv2.destroyAllWindows()
		return


def main():
    LEFT_IR_TOPIC = '/robot/range/left_hand_range/state'
    RIGHT_IR_TOPIC = '/robot/range/right_hand_range/state'
    LEFT_IMAGE_TOPIC = '/cameras/left_hand_camera/image'
    RIGHT_IMAGE_TOPIC = '/cameras/right_hand_camera/image'
    rospy.init_node('chopsticks', anonymous=True)
    process = FingerDetectionProcess(LEFT_IR_TOPIC, RIGHT_IR_TOPIC, LEFT_IMAGE_TOPIC, RIGHT_IMAGE_TOPIC)
    rospy.spin()
    

if __name__ == '__main__':
    main()

  #   image = self.read_image(IMG_DIR + "/test_data.png")
		# image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# image = cv2.GaussianBlur(image, (5,5), 0)

    	

