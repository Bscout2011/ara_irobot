#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>


using namespace std;

class ObstacleAvoidance
{
    public:
        ObstacleAvoidance();
    private:
        void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber odom_sub_;
        float x_pos_;
        float y_pos_;
        float theta_pos_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;

};

ObstacleAvoidance::ObstacleAvoidance()
{
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 20);
    scan_sub_ = nh_.subscribe<>("scan", 20, &ObstacleAvoidance::scanCallBack, this);
    odom_sub_ = nh_.subscribe<>("odom", 20, &ObstacleAvoidance::odomCallBack, this);
}

void ObstacleAvoidance::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    x_pos_ = msg->pose.pose.position.x;
    y_pos_ = msg->pose.pose.position.y;
    theta_pos_= tf::getYaw(msg->pose.pose.orientation);;
}

void ObstacleAvoidance::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{

}