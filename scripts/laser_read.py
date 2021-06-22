#! /usr/bin/env python

# This script is built off a tutorial from https://www.theconstructsim.com/read-laserscan-data/

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from rospy.numpy_msg import numpy_msg

import matplotlib.pyplot as plt


def callback(msg):
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
    num_rays = len(msg.ranges)
    # Zero angle is to the right of sensor
    angles = msg.angle_min + msg.angle_increment * np.arange(num_rays)
    rays = np.array(msg.ranges)
    nan_idx = np.isnan(rays)
    rays[nan_idx] = 5.0

    # Taking distance to right side
    x_distances = rays * np.sin(angles)
    y_distances = rays * np.cos(angles)
    # Output only angles between -pi/2, pi/2
    # quad4_mask = np.logical_and(angles > -np.pi/2, angles < 0)
    # quad1_mask = np.logical_and(angles < np.pi/2, angles > 0)
    right_side_mask = angles < 0
    
    x_rs = x_distances[right_side_mask]  # make distances positive
    y_rs = y_distances[right_side_mask]
    right_side_angles = angles[right_side_mask]

    plt.plot(x_rs, y_rs, "o")
    plt.show()

    print "%.2f, %.2f]" % (np.mean(x_rs), np.mean(y_rs))
    # print np.array2string(angles * 180 / np.pi, precision=2)
    # print "[%.2f, %.2f, %.2f]" % (rays[0], rays[num_rays/2], rays[-1])


rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), callback)
# rospy.spin()