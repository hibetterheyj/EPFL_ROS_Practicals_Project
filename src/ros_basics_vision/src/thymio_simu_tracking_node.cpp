#include <ros_basics_vision/aruco_detector.h>
#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_vision/frame_info.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>

using namespace ros_tp;

struct MouseDetect {
    static void onMouse(int event, int x, int y, int flags, void* param) // now it's in param
    {
        if (event == cv::EVENT_LBUTTONDOWN) {
            cv::Point p(x - camera_px_width / 2, y - camera_px_height / 2);
            geometry_msgs::Pose2D wp;
            wp.x = (p.x * pix2mm) / 1000;
            wp.y = -(p.y * pix2mm) / 1000;

            ros_basics_msgs::AddWaypoint srv;
            srv.request.goal = wp;
            if (!add_waypoint_cli.call(srv)) {
                ROS_ERROR("Failed to add waypoint");
            }
        }
    }

    static double pix2mm;
    static ros::ServiceClient add_waypoint_cli;
    static double camera_px_width;
    static double camera_px_height;
};
double MouseDetect::pix2mm;
ros::ServiceClient MouseDetect::add_waypoint_cli;
double MouseDetect::camera_px_width;
double MouseDetect::camera_px_height;

struct RawFrameWrapper : public FrameInfo {
public:
    RawFrameWrapper(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<image_transport::ImageTransport> it, double pix2mm)
        : FrameInfo(nh, it, pix2mm, 640, 480),
          _window_name("Tracking Robot - Interactive")
    {
        this->_nh->param<bool>("annotate_image", _annotate_image, true);
        this->_nh->param<bool>("use_mouse_waypoints", _use_mouse_waypoints, true);
        if (_use_mouse_waypoints) {
            cv::namedWindow(_window_name);
            MouseDetect::pix2mm = _pix2mm;
            MouseDetect::camera_px_width = 640;
            MouseDetect::camera_px_height = 480;
            MouseDetect::add_waypoint_cli = this->_nh->serviceClient<ros_basics_msgs::AddWaypoint>("add_waypoint");
            cv::setMouseCallback(_window_name, MouseDetect::onMouse);
        }
        _odom_sub = this->_nh->subscribe("robot_pose", 5, &RawFrameWrapper::pose_callback, this);
        _prev_stamp = ros::Time::now();
    }

    void pose_callback(const ros_basics_msgs::SimplePoseStamped::ConstPtr& msg)
    {
        // position
        _pos = cv::Point2f(msg->pose.xyz.x * 1000 / _pix2mm + this->_ccenter.x, 2 * msg->pose.xyz.y * 1000 / _pix2mm + this->_ccenter.y);
        _yaw = msg->pose.rpy.yaw;
    }

    void draw_info(cv::Mat& frame)
    {
        ros::Time now = ros::Time::now();
        double dt = ros::Duration(now - _prev_stamp).toSec();

        cv::Point2f inv_pos(_pos);
        inv_pos.y = 2 * this->_ccenter.y - inv_pos.y;
        this->draw_pose(frame, inv_pos, _yaw);
        this->draw_waypoints(frame);
        this->draw_camera_origin(frame);
        this->draw_axes(frame);
        this->draw_fps(frame, dt);

        _prev_stamp = now;
        this->publish(frame);
    }

    void annot_image_cb(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
            if (_annotate_image) {
                // cv::flip(frame, frame, 0);
                draw_info(frame);
            }
            cv::imshow(_window_name, frame);
            cv::waitKey(10);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

protected:
    std::string _window_name;
    bool _annotate_image;
    bool _use_mouse_waypoints;
    ros::Subscriber _odom_sub;
    cv::Point2f _pos;
    double _yaw;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thymio_simu_tracking_node");
    ros::NodeHandle nh;
    std::shared_ptr<image_transport::ImageTransport> it(new image_transport::ImageTransport(nh));

    double pix2mm;
    nh.param<double>("pix2mm", pix2mm, 1.002142315);

    RawFrameWrapper rfw(std::make_shared<ros::NodeHandle>(nh), it, pix2mm);
    image_transport::Subscriber sub = it->subscribe("robot_view/image_raw", 1, &RawFrameWrapper::annot_image_cb, &rfw);

    ros::spin();

    return 0;
}