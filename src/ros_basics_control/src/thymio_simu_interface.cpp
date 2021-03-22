#include <ros_basics_control/thymio_simu_interface.h>

namespace ros_tp {

    ThymioSimuInterface::ThymioSimuInterface(std::shared_ptr<ros::NodeHandle> nh, bool) : ThymioInterfaceBase(nh, true), _sensor_counter(0)
    {

        _vel_pub = _nh->advertise<geometry_msgs::Twist>("cmd_vel", 1);
        _prox_sensor_pub = _nh->advertise<ros_basics_msgs::ProximitySensors>("proximity_sensors", 1);

        _laser_subs.resize(7);
        _prox_sensor_values.resize(7);
        _laser_subs[0] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_0/scan", 1, &ThymioSimuInterface::_prox_s0_cb, this);
        _laser_subs[1] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_1/scan", 1, &ThymioSimuInterface::_prox_s1_cb, this);
        _laser_subs[2] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_2/scan", 1, &ThymioSimuInterface::_prox_s2_cb, this);
        _laser_subs[3] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_3/scan", 1, &ThymioSimuInterface::_prox_s3_cb, this);
        _laser_subs[4] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_4/scan", 1, &ThymioSimuInterface::_prox_s4_cb, this);
        _laser_subs[5] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_5/scan", 1, &ThymioSimuInterface::_prox_s5_cb, this);
        _laser_subs[6] = nh->subscribe<sensor_msgs::LaserScan>("thymio_laser_6/scan", 1, &ThymioSimuInterface::_prox_s6_cb, this);
    }

    void ThymioSimuInterface::_prox_s0_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 0);
    }

    void ThymioSimuInterface::_prox_s1_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 1);
    }

    void ThymioSimuInterface::_prox_s2_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 2);
    }

    void ThymioSimuInterface::_prox_s3_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 3);
    }

    void ThymioSimuInterface::_prox_s4_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 4);
    }

    void ThymioSimuInterface::_prox_s5_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 5);
    }

    void ThymioSimuInterface::_prox_s6_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        _publish_prox_values(msg->ranges[0], 6);
    }

    void ThymioSimuInterface::_publish_prox_values(double value, int idx)
    {
        const std::lock_guard<std::mutex> lock(_pval_lock);
        if (_sensor_counter == _prox_sensor_values.size()) {
            ros_basics_msgs::ProximitySensors msg;
            msg.header.stamp = ros::Time::now();
            msg.values = _prox_sensor_values;
            _prox_sensor_pub.publish(msg);
            _sensor_counter = 0;
        }
        else {
            _prox_sensor_values[idx] = value;
            ++_sensor_counter;
        }
    }

    void ThymioSimuInterface::set_target_velocities(double v, double z)
    {
        geometry_msgs::Twist tw;
        tw.linear.x = v;
        tw.angular.z = z;
        _vel_pub.publish(tw);
    }

} // namespace ros_tp