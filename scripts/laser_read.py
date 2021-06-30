#! /usr/bin/env python

# This script is built off a tutorial from https://www.theconstructsim.com/read-laserscan-data/

from numpy.lib.financial import rate
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PointStamped
from create_msgs.msg import Bumper


class Robot:

    def __init__(self):
        # Creates a node
        rospy.init_node('laser_control')

        # Publisher to command velocity
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # Subscriber to the /scan when a message of type LaserScan is received
        self.scan_sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), self.compute_free_heading)
        # Publisher for avoid_obstacle heading
        self.heading_error = rospy.Publisher("heading_error", PointStamped, queue_size=10)
        
        self.wheel_drop_sub = rospy.Subscriber("/wheeldrop", Empty, self.wheel_drop)
        # self.bumper_sub = rospy.Subscriber("/bumper", Bumper, self.bumper_cb)

        self.heading = 0
        self.mag = 0
        self.hit = False
        self.bump_time = 0
        self.rate = rospy.Rate(10)  # 10hz
        

    def bumper_cb(self, msg):
        if msg.is_left_pressed:
            rospy.loginfo("Left bumper hit!")
            self.move_back(-1)

        elif msg.is_right_pressed:
            rospy.loginfo("Right bumper hit!")
            self.move_back(1)


    def move_back(self, turn_rad):
        now = rospy.get_time()
        if now - self.bump_time < 2:
            return
        self.bump_time = rospy.get_time()
        self.hit = True
        vel_msg = Twist()

        # Move back 1 sec
        vel_msg.linear.x = -0.2
        vel_msg.angular.z = 0
        while rospy.get_time() - now < 1:
            # rospy.loginfo("Time elapsed: %d", rospy.get_time() - now)
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()

        # Turn for 1 sec
        vel_msg.linear.x = 0
        vel_msg.angular.z = turn_rad
        now = rospy.get_time()
        while rospy.get_time() - now < 1:
            # rospy.loginfo("Time elapsed: %d", rospy.get_time() - now)
            self.vel_pub.publish(vel_msg)
            self.rate.sleep()
        
        self.hit = False


    def wheel_drop(self, msg):
        if msg:
            rospy.signal_shutdown("Wheel Drop. Shutting down.")


    def convert_polar_to_cartesian(self, msg):
        num_rays = len(msg.ranges)
        # Zero angle is in front of sensor
        angles = msg.angle_min + msg.angle_increment * np.arange(num_rays)
        rays = np.array(msg.ranges)
        # Replace nan with range_max
        nan_idx = np.isnan(rays)
        rays[nan_idx] = msg.range_max
        # Convert polar to cartesian coordinates
        points = np.array([rays * np.sin(angles), rays * np.cos(angles)])
        return points, angles
        

    def compute_free_heading(self, scan):
        """Callback function to find a heading towards free space.
        """
        points, angles = self.convert_polar_to_cartesian(scan)
        # Weight each point from least important in front, to most important at the sides
        weights = np.abs(np.sin(angles))
        weights = weights / np.sum(weights)  # make this a convex combination
        # Combine all laser points with their convex weights
        # magnitude = np.sum((1 / points) * weights, axis=1)
        mag = (points * weights).sum(axis=1)
        heading = np.arctan2(mag[0], mag[1])

        self.heading = heading
        self.mag = np.sqrt(np.linalg.norm(mag))
        
        point_msg = PointStamped()
        point_msg.point.x = mag[0]
        point_msg.point.y = mag[1]
        self.heading_error.publish(point_msg)
        # print np.array2string(mag, precision=2), heading * 180 / np.pi

    def angular_vel(self, constant=2):
        return constant * (self.heading)

    def avoid_obstacle(self, fw_vel=0.1):
        """Point robot towards largest free space.
        """
        rospy.loginfo("Running avoid obstacle routine.")
        vel_msg = Twist()
        #We wont use linear components        
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0

        while not rospy.is_shutdown():
            vel_msg.linear.x=fw_vel #* self.mag
            vel_msg.angular.z = self.angular_vel()  # rotate CCW 0.1 radians/sec

            if not self.hit:
                self.vel_pub.publish(vel_msg)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        x = Robot() 
        x.avoid_obstacle(0)   

    except rospy.ROSInterruptException:
        pass