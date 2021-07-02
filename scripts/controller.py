#! /usr/bin/env python

# This script is built off a tutorial from https://www.theconstructsim.com/read-laserscan-data/

from numpy.lib.financial import rate
import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg

import tf

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from create_msgs.msg import Bumper


class Robot:

    def __init__(self, safe_radius=0.15):
        # Creates a node
        rospy.init_node('robot_control')

        self.radius = 0.17  # robot radius in [m]
        self.safe_radius = self.radius + safe_radius  # laser scanner distance to stop when obstacle is detected

        # Publisher to command velocity
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # Subscriber to the /scan when a message of type LaserScan is received
        self.scan_sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), self.convert_polar_to_cartesian)
        # Publisher for avoid_obstacle heading
        self.heading_error = rospy.Publisher("heading_error", PointStamped, queue_size=10)
        # Shutdown if robot is picked up.
        self.wheel_drop_sub = rospy.Subscriber("/wheeldrop", Empty, self.wheel_drop)
        # self.bumper_sub = rospy.Subscriber("/bumper", Bumper, self.bumper_cb)

        self.scan_points = None
        self.scan_angles = None
        self.scan_range = None

        self.heading = 0
        self.mag = 0
        self.hit = False
        self.bump_time = 0
        self.rate = rospy.Rate(10)  # 10hz

        self.publish_safe_radius()
        

    def publish_safe_radius(self):
        vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.header.stamp = rospy.Time()
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.scale.x = self.safe_radius
        marker.scale.y = self.safe_radius
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0

        vis_pub.publish(marker)


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
        # Zero angle is in front of sensor
        if self.scan_angles is None:
            num_rays = len(msg.ranges)
            self.scan_angles = msg.angle_min + msg.angle_increment * np.arange(num_rays)
        rays = np.array(msg.ranges)
        # Replace nan with range_max
        nan_idx = np.isnan(rays)
        rays[nan_idx] = msg.range_max
        # Convert polar to cartesian coordinates
        points = np.array([rays * np.cos(self.scan_angles), rays * np.sin(self.scan_angles)])
        self.scan_points = points
        self.scan_range = rays


    def compute_free_heading(self, scan):
        """Callback function to find a heading towards free space.
        """
        # Weight each point from least important in front, to most important at the sides
        weights = np.abs(np.sin(self.scan_angles))
        weights = weights / np.sum(weights)  # make this a convex combination
        # Combine all laser points with their convex weights
        # magnitude = np.sum((1 / points) * weights, axis=1)
        mag = (self.scan_points * weights).sum(axis=1)
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

    def front_obstacle(self):
        """Return true if an obstacle is in front and within the robot's footprint.
        """
        front_pts = np.logical_and(self.scan_range < self.safe_radius, self.scan_range > self.radius)
        return front_pts.any()


    def avoid_obstacle(self, fw_vel=0.1, angular_vel=0):
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
                self.twist_pub.publish(vel_msg)

            self.rate.sleep()


    def timid(self, fw_vel=0.1):
        rospy.loginfo("Running timid behavior.")

        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.front_obstacle():
                vel_msg.linear.x = 0
            else:
                vel_msg.linear.x=fw_vel 
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()


    def turn_degrees(self, degrees, angular_vel=0.1):
        """Turn robot in place.

        Args:
            degrees (float): CCW is positive. CW is negative
        """
        rospy.loginfo("Turning %f.0 degrees."%degrees)

        twist_msg = Twist()
        initial_odom = rospy.wait_for_message("/odom", Odometry)
        initial_pose = initial_odom.pose

        q1_inv[0] = initial_pose.pose.orientation.x
        q1_inv[1] = initial_pose.pose.orientation.y
        q1_inv[2] = initial_pose.pose.orientation.z
        q1_inv[3] = -initial_pose.pose.orientation.w # Negate for inverse

        angle = degrees * np.pi / 180  # Convert degrees to rad
        t = np.abs(angle / angular_vel)  # time
        direction = np.sign(angle)

        start_time = rospy.Time.now()
        while rospy.Time.now() <= start_time + t:
            twist_msg.angular.z = direction * angular_vel
            self.twist_pub.publish(twist_msg)

        final_odom = rospy.wait_for_message("/odom", Odometry)
        final_pose = final_odom.pose

        q2[0] = final_pose.pose.orientation.x
        q2[1] = final_pose.pose.orientation.y
        q2[2] = final_pose.pose.orientation.z
        q2[3] = final_pose.pose.orientation.w

        qr = tf.transformations.quaternion_multiply(q2, q1_inv)
        euler_r = tf.transformations.euler_from_quaternion(qr)

        rospy.loginfo("Relative quaternion:", qr)
        rospy.loginfo("Relative euler:", euler_r)

        
        



if __name__ == "__main__":
    try:
        x = Robot() 
        x.turn_degrees(90)   

    except rospy.ROSInterruptException:
        pass