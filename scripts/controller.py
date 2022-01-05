#! /usr/bin/env python

# This script is built off a tutorial from https://www.theconstructsim.com/read-laserscan-data/

import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg

from os import path

import tf

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from create_msgs.msg import Bumper, Cliff


DEG2RAD = np.pi / 180.0
RAD2DEG = 180 / np.pi
DATA_DIRECTORY = "/home/aralab/roomba_ws/results"


class Robot:

    def __init__(self, safe_radius=0.15):
        # Creates a node
        rospy.init_node('robot_control')

        # Define variables
        self.L = 0.235  # wheel base width
        self.radius = 0.17  # robot radius in [m]
        self.safe_radius = self.radius + safe_radius  # laser scanner distance to stop when obstacle is detected

        # Processed Laser Scan points
        self.scan_points = None
        self.scan_angles = None
        self.scan_range = None
        self.object_sector = np.array([0.0, 0.0, 0.0])
        self.object_range = 2.0

        self.heading = 0.0
        self.prev_heading = 0.0
        self.mag = 0
        self.hit = False
        self.bump_time = 0
        self.rate = rospy.Rate(10)  # 10hz

        # Publishers 
        self.twist_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.heading_error = rospy.Publisher("heading_error", PointStamped, queue_size=10)

        # Subscribers 
        self.scan_sub = rospy.Subscriber('/scan', numpy_msg(LaserScan), self.laser_cb)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.bumper_sub = rospy.Subscriber("/bumper", Bumper, self.bumper_cb)
        self.cliff_sub = rospy.Subscriber("/cliff", Cliff, self.cliff_cb)
        # Shutdown if robot is picked up.
        self.wheel_drop_sub = rospy.Subscriber("/wheeldrop", Empty, self.wheel_drop)
        
        rospy.wait_for_message("/odom", Odometry)
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
        pass

    def cliff_cb(self, msg):
        pass

    def odom_cb(self, msg):
        self.odom = msg

    def update_velocity(self, v, omega):
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = omega
        self.twist_pub.publish(vel)

    def wheel_drop(self, msg):
        if msg:
            rospy.signal_shutdown("Wheel Drop. Shutting down.")

    def laser_cb(self, msg):
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
        self.sector_min_range()


    def sector_min_range(self):
        """Divide front halfspace into 3 sectors.
        Sets self.object_sector with minimum laser scan range.
        """
        front_pts = self.scan_range > self.radius  # omit points within the robot's footprint
        # divide points into 3 regions of 60 deg
        bins = [self.scan_angles.max(), np.pi/6, -np.pi/6, self.scan_angles.min()]
        sectors = np.digitize(self.scan_angles, bins)[front_pts]
        # for each sector, get the average distance of the minimum 3 points
        min_elements = 3
        for i in range(1, len(bins)):
            sector_ranges = self.scan_range[front_pts][sectors == i]
            sector_min_elements = np.partition(sector_ranges, min_elements)[:min_elements]
            self.object_sector[i-1] = sector_min_elements.mean()

    def object_near(self):
        """Return true if an obstacle is within the robot's footprint.
        """
        return (self.object_sector < self.safe_radius).any()


    def timid(self, fw_vel=0.1):
        """Move forward until detect object within safe radius.
        """
        rospy.loginfo("Running timid behavior.")

        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.front_obstacle():
                vel_msg.linear.x = 0
            else:
                vel_msg.linear.x=fw_vel 
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()


    def indecisive(self, fw_vel=0.1):
        """Move forward until detect object within safe radius,
        then move backward until object out of safe radius.
        """
        rospy.loginfo("Running indecisive behavior.")

        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.front_obstacle():
                vel_msg.linear.x = -fw_vel
            else:
                vel_msg.linear.x = fw_vel 
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()


    def dogged(self, fw_vel=0.1):
        """Move away from object in front or behind.
        """
        rospy.loginfo("Running dogged behavior.")

        vel_msg = Twist()

        while not rospy.is_shutdown():
            if self.front_obstacle():
                vel_msg.linear.x = -fw_vel
            else:
                vel_msg.linear.x = 0 
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()


    def paranoid(self, fw_vel=0.1, angular_vel=0.5):            
        rospy.loginfo("Running paranoid behavior")

        vel_msg = Twist()

        while not rospy.is_shutdown():
            
            sector = self.obstacle_sector()
            print sector

            if sector == 0:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
            elif sector == 1:
                vel_msg.linear.x = 0
                vel_msg.angular.z = angular_vel
            elif sector == 2:
                vel_msg.linear.x = fw_vel
                vel_msg.angular.z = 0
            elif sector == 3:
                vel_msg.linear.x = 0
                vel_msg.angular.z = -angular_vel
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()

    def paranoid_state(self, fw_vel=0.5, angular_vel=0.5):
        """State based paranoid behavior. 
        - When an object is detected in front of the robot, the robot moves forwards.
        - When an object is detected to the right of the robot, the robot turns right.
        - When an object is detected to the left of the robot, the robot turns left.
        - If the robot is turning (even if it no longer detects an object), 
            it alternates the direction of its turn every second.
        - When no object is detected and the robot is not turning, the robot stops.

        Args:
            fw_vel (float, optional): Forward velocity. Defaults to 0.5.
            angular_vel (float, optional): Turn velocity. Defaults to 0.5.
        """
        state = "Search"

        direction = 1
        turn_time = 1
        turn_start_time = rospy.get_time()

        while not rospy.is_shutdown():
            print state
            if state == "Search":
                if self.object_sector[1] < self.object_range:
                    state = "Forward"
                elif self.object_sector[0] < self.object_range:
                    state = "Left"
                elif self.object_sector[2] < self.object_range:
                    state = "Right"
                else:
                    # Search alternating left / right every 1 second.
                    if turn_start_time + turn_time < rospy.get_time():
                        # switch direction
                        turn_start_time = rospy.get_time()
                        direction *= -1
                    self.update_velocity(0, direction * angular_vel)
                    
            elif state == "Left":
                if self.object_sector[1] < self.object_range:
                    state = "Forward"
                else:
                    self.update_velocity(0, angular_vel)

            elif state == "Right":
                if self.object_sector[1] < self.object_range:
                    state = "Forward"
                else:
                    self.update_velocity(0, -angular_vel)

            elif state == "Forward":
                if self.object_near():
                    print "Object in Front"
                    state = "Found"
                else:
                    self.update_velocity(fw_vel, 0)

            elif state == "Found":
                self.update_velocity(0, 0)
                if not self.object_near():
                    print "No object near. Starting search."
                    state = "Search"
            else:
                raise NotImplementedError("State [%s] not implemented"%state)

            self.rate.sleep()


    def insecure(self, fw_vel=0.1):
        """Turn away from object on the left.
        """
        rospy.loginfo("Running insecure behavior.")

        vel_msg = Twist()
        v = fw_vel / 2
        omega = fw_vel / self.L

        while not rospy.is_shutdown():
            if self.object_left():
                # Object detected on the left. Turn right
                vel_msg.linear.x = v
                vel_msg.angular.z = -omega
            else:
                # No object on left. Turn left
                vel_msg.linear.x = v
                vel_msg.angular.z = omega
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()

    def driven(self, fw_vel=0.1):
        """Turn towards from object on the left.
        """
        rospy.loginfo("Running driven behavior.")

        vel_msg = Twist()
        v = fw_vel / 2
        omega = fw_vel / self.L

        while not rospy.is_shutdown():
            if self.object_left():
                # Object detected on the left. Turn right
                vel_msg.linear.x = v
                vel_msg.angular.z = omega
            else:
                # No object on left. Turn left
                vel_msg.linear.x = v
                vel_msg.angular.z = -omega
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()


    def turn_degrees(self, degrees, angular_vel=1):
        """Turn robot in place.

        Args:
            degrees (float): CCW is positive. CW is negative
        """
        twist_msg = Twist()
        initial_odom = rospy.wait_for_message("/odom", Odometry)
        initial_pose = initial_odom.pose

        q1_inv = np.zeros(4)
        q1_inv[0] = initial_pose.pose.orientation.x
        q1_inv[1] = initial_pose.pose.orientation.y
        q1_inv[2] = initial_pose.pose.orientation.z
        q1_inv[3] = -initial_pose.pose.orientation.w # Negate for inverse

        angle = degrees * np.pi / 180  # Convert degrees to rad
        t = np.abs(angle / angular_vel)  # time
        direction = np.sign(angle)

        start_time = rospy.get_time()
        while rospy.get_time() <= start_time + t:
            twist_msg.angular.z = direction * angular_vel
            self.twist_pub.publish(twist_msg)
            
            self.rate.sleep()

        final_odom = rospy.wait_for_message("/odom", Odometry)
        final_pose = final_odom.pose

        q2 = np.zeros(4)
        q2[0] = final_pose.pose.orientation.x
        q2[1] = final_pose.pose.orientation.y
        q2[2] = final_pose.pose.orientation.z
        q2[3] = final_pose.pose.orientation.w

        qr = tf.transformations.quaternion_multiply(q2, q1_inv)
        euler_r = tf.transformations.euler_from_quaternion(qr)
        odom_angle = euler_r[2] * 180 / np.pi
        error = degrees - odom_angle

        rospy.loginfo("Relative odom error: %.1f"%error)
        return error

    def line_following(self, fw_vel=0.5, ang_vel=1, sensor="left"):
        """Implement a line following robot.

        Args:
            fw_vel (float, optional): Forward velocity. Defaults to 0.5.
            ang_vel (float, optional): Angular turning velocity. Defaults to 1.
            sensor (string, optional): which sensor to use for line following. 
                    Options are ('left', 'front-left', 'front-right', 'right').
                    Defaults to 'left'.
        """
        sensor_thresholds = {
            "left":         (2650, 2750),
            "front-left":   (2750, 2825),
            "front-right":  (2650, 2750),
            "right":        (2700, 2750)
        }

        if sensor not in sensor_thresholds:
            raise KeyError("Invalid sensor passed.")

        thres = sensor_thresholds[sensor]

        vel_msg = Twist()
        v = fw_vel
        omega = ang_vel

        while not rospy.is_shutdown():
            if self.object_left():
                # Object detected on the left. Turn right
                vel_msg.linear.x = v
                vel_msg.angular.z = omega
            else:
                # No object on left. Turn left
                vel_msg.linear.x = v
                vel_msg.angular.z = -omega
            
            self.twist_pub.publish(vel_msg)
            self.rate.sleep()
        
    def forward_dist(self, x=1, v=0.5):
        """Drive forward `x` meters at `v` m/s speed.
        v [float]: velocity. Experimental max is 0.35 m/s
        """
        if v > 0.5:
            v = 0.5
            
        rospy.loginfo("Driving forward %.1f [m] at %.2f [m/s]."%(x, v))
        t_start = rospy.get_time()
        dt = x / v
        poses = []
        twists = []
        while rospy.get_time() <= t_start + dt:
            self.update_velocity(v, 0)
            poses.append(self.odom.pose.pose)
            twists.append(self.odom.twist.twist)
            
            self.rate.sleep()
        # Get two time stamps after time stop
        for _ in range(5):
            self.update_velocity(0, 0)
            poses.append(self.odom.pose.pose)
            twists.append(self.odom.twist.twist)
            self.rate.sleep()

        # Save position data
        odom = [(p.position.x, p.position.y, v.linear.x, v.linear.y) for p, v in zip(poses, twists)]
        odom = np.array(odom)

        fn = "odom_" + str(t_start) + ".npy"
        with open(path.join(DATA_DIRECTORY, fn), 'wb') as f:
            np.save(f, odom)



        


def stationary_rotation(angle, speed, n_trials):

    try:
        # rospy.loginfo("Turning %.0f degrees in %.1f seconds."%(degrees, t))
        x = Robot() 
        for _ in range(n_trials):
            x.turn_degrees(angle, speed)   
            x.turn_degrees(-angle, speed)
    
    except rospy.ROSInterruptException:
        pass
    
def odometry_trial(dist=1, speed=0.5):
    try:
        x = Robot()
        for _ in range(3):
            x.forward_dist(dist, speed)
            rospy.sleep(.5)
            x.turn_degrees(180)
            rospy.sleep(.5)
            x.forward_dist(dist, speed)
            x.turn_degrees(-180)
            rospy.sleep(.5)

    except rospy.ROSInterruptException:
        pass

def behavior(name):
    try:
        x = Robot()
        
        if name == "indecisive":
            x.indecisive()
        elif name == "timid":
            x.timid()
        elif name == "dogged":
            x.dogged()
        elif name == "paranoid":
            x.paranoid()
        elif name == "insecure":
            x.insecure()
        elif name == "driven":
            x.driven()
        elif name == "paranoid_state":
            x.paranoid_state()
        else:
            print "Behavior not implemented"


    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    # stationary_rotation(90, 2, 4)
    # behavior("paranoid_state")
    odometry_trial(2, .33)
    
    