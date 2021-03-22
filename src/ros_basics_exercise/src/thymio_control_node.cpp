#include "ros/ros.h"

#include <ros_basics_control/thymio.hpp>
#include <ros_basics_exercise/pid_controller.hpp>
#include <ros_basics_control/thymio_simu_interface.h>
#include <ros_basics_control/robot_tracker.h>
#include <ros_basics_control/gazebo_robot_tracker.h>

#include <iostream>
#include <signal.h>

using namespace ros_tp;

struct Params {
    struct ControllerParams : public defaults::ControllerParams {
    };
};

template <typename T>
void thymio_spin(T& th)
{
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        th.spin_control(false);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thymio_control_ex_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());

    bool is_simu;
    nh->param<bool>("is_simu", is_simu, true);

    if (!is_simu) {
        Thymio<ThymioInterfaceBase, PIDController<Params>, RobotTracker> th(nh);
        thymio_spin(th);
    }
    else {
        Thymio<ThymioSimuInterface, PIDController<Params>, GazeboRobotTracker> th(nh);
        thymio_spin(th);
    }

    return 0;
}