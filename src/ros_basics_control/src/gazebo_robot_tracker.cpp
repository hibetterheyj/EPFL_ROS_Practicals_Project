#include <ros_basics_control/gazebo_robot_tracker.h>

#include <ros/ros.h>
#include <tf/tf.h>

namespace ros_tp {
    GazeboRobotTracker::GazeboRobotTracker(std::shared_ptr<ros::NodeHandle> nh) : _nh(nh)
    {
        _odom_sub = _nh->subscribe("odom", 1, &GazeboRobotTracker::_odom_callback, this);
        _pose_pub = nh->advertise<ros_basics_msgs::SimplePoseStamped>("robot_pose", 1);
    }

    void GazeboRobotTracker::_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // position
        _pos.x = msg->pose.pose.position.x;
        _pos.y = msg->pose.pose.position.y;
        _pos.z = msg->pose.pose.position.z;

        // orientation, from quaternions to roll, pitch, yaw
        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        m.getRPY(_rpy.roll, _rpy.pitch, _rpy.yaw);

        // publish pose in the exercise's format
        ros_basics_msgs::SimplePoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.pose.xyz.x = _pos.x;
        pose.pose.xyz.y = _pos.y;
        pose.pose.xyz.z = _pos.z;
        pose.pose.rpy.roll = _rpy.roll;
        pose.pose.rpy.pitch = _rpy.pitch;
        pose.pose.rpy.yaw = _rpy.yaw;
        _pose_pub.publish(pose);
    }

} // namespace ros_tp