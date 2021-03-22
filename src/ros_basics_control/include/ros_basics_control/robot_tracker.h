#ifndef ROS_TP_ROBOT_TRACKER_H
#define ROS_TP_ROBOT_TRACKER_H

#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_msgs/RPY.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
namespace ros_tp {

    class RobotTracker {
    public:
        RobotTracker();
        RobotTracker(std::shared_ptr<ros::NodeHandle>);

        using Pose = ros_basics_msgs::SimplePoseStamped;
        using Pos = geometry_msgs::Point;
        using RPY = ros_basics_msgs::RPY;

        virtual Pose get_pose() const;
        virtual Pos get_position() const;
        virtual RPY get_orientation() const;

    protected:
        Pos _pos;
        RPY _rpy;
    };

} // namespace ros_tp

#endif