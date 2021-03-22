#ifndef ROS_TP_THYMIO_H
#define ROS_TP_THYMIO_H

#include <ros/ros.h>
#include <ros_basics_msgs/SimplePoseStamped.h>

namespace ros_tp {

    template <typename ThymioInterface, typename Controller, typename RobotTracker>
    class Thymio {
    public:
        Thymio(std::shared_ptr<ros::NodeHandle> nh, bool advertise = true) : _nh(nh),
                                                                             _ctrl(nh),
                                                                             _ti(nh, advertise),
                                                                             _tracker(_nh) {}

        void spin_control(bool verbose = false)
        {
            _ctrl.move(_tracker.get_pose(), _ti, verbose);
        }

        Controller& get_controller()
        {
            return _ctrl;
        }

        RobotTracker& get_robot_tracker()
        {
            return _tracker;
        }

        ThymioInterface& get_thymio_interface()
        {
            return _ti;
        }

    protected:
        std::shared_ptr<ros::NodeHandle> _nh;

        Controller _ctrl;
        ThymioInterface _ti;
        RobotTracker _tracker;

        ros_basics_msgs::SimplePoseStamped _pose;
    };

} // namespace ros_tp

#endif