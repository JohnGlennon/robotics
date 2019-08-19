#!/usr/bin/env python
#
#    eindproject.py - Glenn Geysen and Mario van Kerckhoven
#
#    The robot moves and follows some traffic rules.
#	 The robot stops in front of a red light and it follows colors that give direction.
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image
from math import isnan
from std_msgs.msg import String
import numpy as np


class Car():
	def __init__(self):
		rospy.init_node("car")

		# Set the shutdown function
		rospy.on_shutdown(self.shutdown)

		# Initialise the laser
		self.laser = None

		# The speed in meters per second
		self.speed = 0.4

		# The angle in case the turtlebot needs to turn
		self.angle = 90

		self.PI = 3.1415926535897

		# The speed of turning
		self.angular_speed = 1

		# Converting the angle in radians
		self.relative_angle = self.angle * 2 * self.PI / 360

		# Converts image to bgr
		self.bridge = CvBridge()

		# Subscribe to image topic
		self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)

		# Checks if an image is received
		self.image_received = False

		self.image = None

		# Set rate to update robot's movement
		self.rate = rospy.Rate(10)

		# Initialize the movement command
		self.move_cmd = Twist()

		# Publisher to control the robot's movement
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

		# Distance to wall
		self.distance = 0

		# Subscribe to the laserscan
		self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

		rospy.loginfo("Subscribing to laserscan...")

		# Wait for the laserscan topic to become available
		rospy.wait_for_message('/scan', LaserScan)

		rospy.loginfo("Ready to start!")

	def laserscan_callback(self, msg):
		self.laser = msg

		n = 0

		midrange_min = midrange_max = 0

		# Get indexes to filter the "nbr_mid" middle point out of the laserscan points
		len_scan = len(msg.ranges)
		nbr_mid = 50
		if (len_scan > nbr_mid):
			midrange_min = int((len_scan - nbr_mid) / 2)
			midrange_max = midrange_min + nbr_mid

		# Compute average distance out of the middle points, skip NaN
		for point in msg.ranges[midrange_min:midrange_max]:
			if not isnan(point):
				self.distance += point
				n += 1

		# If no points, keep dist equal to zero
		if n:
			self.distance /= n
		rospy.loginfo("dist: %s, nbr of points %s", String(self.distance), String(n))

	def image_callback(self, data):
		# Convert image to OpenCV format
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.loginfo(e)

		self.image_received = True
		self.image = cv_image

	def start(self):
		rospy.loginfo("start Car Node")

		while not rospy.is_shutdown():

			# Move the turtlebot
			self.move_cmd.linear.x = self.speed

			# Take a photo

			# Use '_image_title' parameter from command line
			# Default value is 'photo.jpg'
			img_title = rospy.get_param('~image_title', 'photo.jpg')

			if car.take_picture(img_title):
				rospy.loginfo("Saved image " + img_title)
			else:
				rospy.loginfo("No images received")

			# Publish the movement command
			self.cmd_vel_pub.publish(self.move_cmd)
			self.rate.sleep()

	def take_picture(self, img_title):
		if self.image_received:
			# Save an image
			cv2.imwrite(img_title, self.image)

			img = cv2.imread('photo.jpg', 1)
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

			# Range for lower red
			lower_red = np.array([0, 120, 70], dtype=np.uint8)
			upper_red = np.array([10, 255, 255], dtype=np.uint8)
			mask1 = cv2.inRange(hsv, lower_red, upper_red)

			# Range for upper red
			lower_red = np.array([170, 120, 70], dtype=np.uint8)
			upper_red = np.array([180, 255, 255], dtype=np.uint8)
			mask2 = cv2.inRange(hsv, lower_red, upper_red)

			# Generating the final mask to detect red
			mask = mask1 + mask2

			cv2.destroyAllWindows()

			if cv2.countNonZero(mask) == 0:
				rospy.loginfo("Image doesn't have the required color")
			elif self.distance > 2 or self.distance < 1.5:
				rospy.loginfo("Color detected but distance is too long or too short")
			else:
				rospy.loginfo("Waits 5 seconds for the red light")
				self.move_cmd.linear.x = 0
				self.move_cmd.angular.z = 0
				rospy.sleep(5)

			# Lower green
			lower_green = np.array([65, 60, 60], dtype=np.uint8)

			# Upper green
			upper_green = np.array([80, 255, 255], dtype=np.uint8)

			# Generating the mask to detect green
			mask_green = cv2.inRange(hsv, lower_green, upper_green)

			if cv2.countNonZero(mask_green) == 0:
				rospy.loginfo("Image doesn't have the color green")
			elif self.distance > 1.25:
				rospy.loginfo("Color green detected but distance is too long")
			else:
				rospy.loginfo("Turning 90 degrees to the right")
				self.rotate("right")

			return True
		else:
			return False

	def rotate(self, direction):
		if direction == "right":
			self.move_cmd.angular.z = -self.angular_speed
		elif direction == "left":
			self.move_cmd.angular.z = self.angular_speed

		# Setting the current time for distance calculus
		t0 = rospy.Time.now().to_sec()
		current_angle = 0

		while current_angle < self.relative_angle:
			rospy.loginfo(current_angle)
			self.cmd_vel_pub.publish(self.move_cmd)
			t1 = rospy.Time.now().to_sec()
			current_angle = self.angular_speed * (t1 - t0)

		# Stop turning
		self.move_cmd.angular.z = 0

	def shutdown(self):
		rospy.loginfo("Stopping the robot...")

		# Unregister the subscriber to stop cmd_vel publishing
		self.scan_subscriber.unregister()
		rospy.sleep(1)

		# Send an emtpy Twist message to stop the robot
		self.cmd_vel_pub.publish(Twist())
		rospy.sleep(1)


if __name__ == '__main__':
	try:
		car = Car()
		car.start()
	except rospy.ROSInterruptException:
		rospy.loginfo("Car node terminated.")
