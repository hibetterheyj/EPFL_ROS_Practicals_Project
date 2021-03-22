#include <ros_basics_control/robot_tracker.h>

namespace ros_tp {
    RobotTracker::RobotTracker() {}
    RobotTracker::RobotTracker(std::shared_ptr<ros::NodeHandle>) {}

    RobotTracker::Pose RobotTracker::get_pose() const
    {
        Pose pose;
        pose.pose.xyz.x = _pos.x;
        pose.pose.xyz.y = _pos.y;
        pose.pose.xyz.z = _pos.z;
        pose.pose.rpy.roll = _rpy.roll;
        pose.pose.rpy.pitch = _rpy.pitch;
        pose.pose.rpy.yaw = _rpy.yaw;
        pose.header.stamp = ros::Time::now();
        return pose;
    }

    RobotTracker::Pos RobotTracker::get_position() const
    {
        return _pos;
    };

    RobotTracker::RPY RobotTracker::get_orientation() const
    {
        return _rpy;
    };

} // namespace ros_tp