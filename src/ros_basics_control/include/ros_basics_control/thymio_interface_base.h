#ifndef ROS_TP_THYMIO_INTERFACE_BASE_H
#define ROS_TP_THYMIO_INTERFACE_BASE_H

#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_msgs/SimpleVelocities.h>

#include <ros/ros.h>

namespace ros_tp {

    class ThymioInterfaceBase {
    public:
        using Pose = ros_basics_msgs::SimplePoseStamped;

        ThymioInterfaceBase(std::shared_ptr<ros::NodeHandle> nh, bool advertise);

        virtual void set_target_velocities(double v, double z);

    protected:
        void _set_velocities_cb(const ros_basics_msgs::SimpleVelocities::ConstPtr& msg);

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::Subscriber _set_velocities_sub;
        bool _advertise;
    };

} // namespace ros_tp

#endif