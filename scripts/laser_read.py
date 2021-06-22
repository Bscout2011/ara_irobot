#! /usr/bin/env python

# This script is built off a tutorial from https://www.theconstructsim.com/read-laserscan-data/

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rospy.numpy_msg import numpy_msg


"""
Single scan from a planar laser range-finder

Header header
stamp: The acquisition time of the first ray in the scan.
frame_id: The laser is assumed to spin around the positive Z axis
(counterclockwise, if Z is up) with the zero angle forward along the x axis

float32 angle_min # start angle of the scan [rad]
float32 angle_max # end angle of the scan [rad]
float32 angle_increment # angular distance between measurements [rad]

float32 time_increment # time between measurements [seconds] - 
if your scanner is moving, this will be used in interpolating position of 3d points
float32 scan_time # time between scans [seconds]

float32 range_min # minimum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities # intensity data [device-specific units]. If your
device does not provide intensities, please leave the array empty.
"""

class Robot:

    def __init__(self):
        # Creates a node
        rospy.init_node('laser_control')

        # Publisher to command velocity
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # Subscriber to the /scan when a message of type LaserScan is received
        self.scan_sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), self.compute_free_heading)
        
        self.heading = 0
        self.rate = rospy.Rate(10)  # 10hz


    def compute_free_heading(self, msg):
        """Callback function to find a heading towards free space.
        """
        num_rays = len(msg.ranges)
        # Zero angle is in front of sensor
        angles = msg.angle_min + msg.angle_increment * np.arange(num_rays)
        rays = np.array(msg.ranges)
        # Replace nan with range_max
        nan_idx = np.isnan(rays)
        rays[nan_idx] = msg.range_max
        # Convert polar to cartesian coordinates
        points = np.array([rays * np.sin(angles), rays * np.cos(angles)])
        # Weight each point from most important in front, to least important at the sides
        weights = np.cos(angles)
        weights = weights / np.sum(weights)  # make this a convex combination
        # Combine all laser points with their convex weights
        # magnitude = np.sum((1 / points) * weights, axis=1)
        mag = (points * weights).sum(axis=1)
        heading = np.arctan2(mag[0], mag[1])

        self.heading = heading
        # print np.array2string(mag, precision=2), heading * 180 / np.pi

    def angular_vel(self, constant=1):
        return constant * (self.heading)

    def avoid_obstacle(self):
        """Point robot towards largest free space.
        """
        rospy.loginfo("Running avoid obstacle routine.")
        vel_msg = Twist()
        #We wont use linear components
        vel_msg.linear.x=0
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        while not rospy.is_shutdown():

            vel_msg.angular.z = self.angular_vel()  # rotate CCW 0.1 radians/sec

            self.vel_pub.publish(vel_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        x = Robot() 
        x.avoid_obstacle()   

    except rospy.ROSInterruptException:
        pass