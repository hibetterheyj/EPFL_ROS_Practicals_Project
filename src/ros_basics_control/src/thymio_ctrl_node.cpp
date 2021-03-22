#include "ros/ros.h"
#include <ros_basics_control/thymio.hpp>

#include <ros_basics_control/thymio_interface_base.h>
#include <ros_basics_control/controller_base.hpp>
#include <ros_basics_control/gazebo_robot_tracker.h>

#include <iostream>

using namespace ros_tp;

struct Params {
    struct ControllerParams : public defaults::ControllerParams {
        static constexpr double precision = 0.01;
    };
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thymio_ctrl_node");
    ros::NodeHandle nh;

    ControllerBase<Params> ctrl_base(std::make_shared<ros::NodeHandle>(nh));

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}