#ifndef ROS_TP_GAZEBO_ROBOT_TRACKER_H
#define ROS_TP_GAZEBO_ROBOT_TRACKER_H

#include "ros/ros.h"
#include <ros_basics_control/robot_tracker.h>
#include <nav_msgs/Odometry.h>

namespace ros_tp {

    class GazeboRobotTracker : public RobotTracker {
    public:
        GazeboRobotTracker(std::shared_ptr<ros::NodeHandle> nh);

    protected:
        void _odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Subscriber _odom_sub;
        ros::Publisher _pose_pub;
    };

} // namespace ros_tp

#endif