#ifndef ROS_TP_THYMIO_SIMU_INTERFACE_BASE_H
#define ROS_TP_THYMIO_SIMU_INTERFACE_BASE_H

#include <ros_basics_control/thymio_interface_base.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <ros_basics_msgs/ProximitySensors.h>

#include <mutex>

namespace ros_tp {

    class ThymioSimuInterface : public ThymioInterfaceBase {
    public:
        ThymioSimuInterface(std::shared_ptr<ros::NodeHandle> nh, bool);

        virtual void set_target_velocities(double v, double z);

    protected:
        void _prox_s0_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _prox_s1_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _prox_s2_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _prox_s3_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _prox_s4_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _prox_s5_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _prox_s6_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
        void _publish_prox_values(double value, int idx);
        std::atomic<size_t> _sensor_counter;

        ros::Publisher _vel_pub;
        ros::Publisher _prox_sensor_pub;
        std::vector<ros::Subscriber> _laser_subs;
        std::vector<double> _prox_sensor_values;

        std::mutex _pval_lock;
    };

} // namespace ros_tp

#endif