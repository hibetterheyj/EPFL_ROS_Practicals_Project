#ifndef ROS_TP_PID_CONTROLLER_EX_H
#define ROS_TP_PID_CONTROLLER_EX_H

#include <ros_basics_control/controller_base.hpp>
#include <ros_basics_msgs/SimplePoseStamped.h>

namespace ros_tp {
    template <typename Params>
    class PIDController {
    public:
        using Pose = ros_basics_msgs::SimplePoseStamped;

        PIDController(std::shared_ptr<ros::NodeHandle> nh) : _nh(nh)

        {
        }

        template <typename ThymioInterface>
        void move(const Pose&, ThymioInterface& ti, bool verbose = true)
        {
            // 1) call the corresponding service to check if the waypoint was reached
            // 2) subscribe to the robot_pose topic to get the robot's current pose
            // 3) call the corresponding service to get the current waypoint or waypoint list
            // 4) implement your PID/PD/P logic
            // 5) publish your computed velocities in the set_velocities topic
            // 6) if there are no waypoints left then set the velocities to 0 and wait for the next waypoint

            // if you are using python then leave this function call empty !
        }

    protected:
        std::shared_ptr<ros::NodeHandle> _nh;
        Pose _pose;
    };

} // namespace ros_tp

#endif