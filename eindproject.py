#!/usr/bin/env python
#
#    eindproject.py - Glenn Geysen and Mario van Kerckhoven
#
#    The robot moves and follows some traffic rules.
#	 The robot stops in front of a red light and it follows arrows that give direction.

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import isnan
from std_msgs.msg import String


class Car():
    def __init__(self):
        rospy.init_node("car")

        # Set the shutdown function
        rospy.on_shutdown(self.shutdown)

        # The speed in meters per second
        self.speed = 0.1

        # Set rate to update robot's movement
        r = rospy.Rate(10)

        # Initialize the movement command
        self.move_cmd = Twist()

        # Publisher to control the robot's movement
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)

        # Subscribe to the laserscan
        self.scan_subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_callback)

        rospy.loginfo("Subscribing to laserscan...")

        # Wait for the laserscan topic to become available
        rospy.wait_for_message('/scan', LaserScan)

        rospy.loginfo("Ready to start!")

        while not rospy.is_shutdown():
            # Publish the movement command
            self.cmd_vel_pub.publish(self.move_cmd)
            r.sleep()

    def laserscan_callback(self, msg):
        dist = n = 0

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
                dist += point
                n += 1

        # If no points, keep dist equal to zero
        if n:
            dist /= n
        rospy.loginfo("dist: %s, nbr of points %s", String(dist), String(n))

        # Move the turtlebot
        self.move_cmd.linear.x = self.speed


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
        Car()
    except rospy.ROSInterruptException:
        rospy.loginfo("Car node terminated.")
