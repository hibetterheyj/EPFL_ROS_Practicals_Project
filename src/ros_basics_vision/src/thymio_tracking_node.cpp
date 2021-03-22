#include <ros_basics_vision/aruco_detector.h>
#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_vision/frame_info.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "thymio_tracking_node");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    std::shared_ptr<image_transport::ImageTransport> it(new image_transport::ImageTransport(*nh));

    int camera_dev;
    nh->param<int>("camera_dev", camera_dev, 0);
    cv::VideoCapture camera(camera_dev);
    ROS_INFO("Using camera device %d", camera_dev);
    if (!camera.isOpened()) {
        ROS_ERROR("ERROR: Could not open camera");
        return 1;
    }

    int camera_px_width, camera_px_height;
    double pix2mm;
    nh->param<int>("camera_px_width", camera_px_width, 640);
    nh->param<int>("camera_px_height", camera_px_height, 480);
    nh->param<double>("pix2mm", pix2mm, 1.002142315);

    camera.set(cv::CAP_PROP_FRAME_WIDTH, camera_px_width);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, camera_px_height);

    image_transport::Publisher raw_image_pub = it->advertise("robot_view/image_raw", 1);
    bool annotate_image, use_mouse_waypoints;
    nh->param<bool>("annotate_image", annotate_image, true);
    nh->param<bool>("use_mouse_waypoints", use_mouse_waypoints, true);
    if (use_mouse_waypoints) {
        std::string name = "Tracking Robot - Interactive";
        cv::namedWindow(name);
        MouseDetect::pix2mm = pix2mm;
        MouseDetect::camera_px_width = camera_px_width;
        MouseDetect::camera_px_height = camera_px_height;
        MouseDetect::add_waypoint_cli = nh->serviceClient<ros_basics_msgs::AddWaypoint>("add_waypoint");
        cv::setMouseCallback(name, MouseDetect::onMouse);
        MouseDetect::add_waypoint_cli.waitForExistence();
    }

    ros::Publisher robot_pose_pub;
    robot_pose_pub = nh->advertise<ros_basics_msgs::SimplePoseStamped>("robot_pose", 1);

    cv::Mat frame;
    std::shared_ptr<ArucoDetector> ad(new ArucoDetector());
    std::vector<float> camera_mat, distortion_coeffs;
    nh->getParam("camera_matrix", camera_mat);
    nh->getParam("distortion_coeffs", distortion_coeffs);
    if (camera_mat.size() && distortion_coeffs.size()) {
        cv::Mat cm(cv::Size(3, 3), CV_32F, camera_mat.data());
        cv::Mat dc(cv::Size(1, 5), CV_32F, distortion_coeffs.data());
        ad->set_camera_matrix(cm);
        ad->set_distortion_coeffs(dc);
        ROS_INFO("Camera coeffs set");
    }

    FrameInfo fi(nh, ad, it, pix2mm, camera_px_width, camera_px_height);
    ros::Rate loop_rate(30);
    while (ros::ok()) {
        camera >> frame;

        // publish raw image
        sensor_msgs::ImagePtr raw_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        raw_image_pub.publish(raw_msg);

        // detect aruco markers
        ad->detect(frame);

        std::vector<cv::Vec6d> poses = ad->get_current_poses();
        if (poses.size()) {
            // publish robot pose if a robot was detected
            ros_basics_msgs::SimplePoseStamped pose; // ! we only use the first robot (no multi robot support for now)
            pose.header.stamp = ros::Time::now();
            pose.pose.xyz.x = poses[0][0];
            pose.pose.xyz.y = poses[0][1];
            pose.pose.xyz.z = poses[0][2];
            pose.pose.rpy.roll = poses[0][3];
            pose.pose.rpy.pitch = poses[0][4];
            pose.pose.rpy.yaw = poses[0][5];
            robot_pose_pub.publish(pose);
        }

        // draw info on the frame
        if (annotate_image) {
            fi.draw_info(frame);
            if (use_mouse_waypoints) {
                cv::imshow("Tracking Robot - Interactive", frame);
                cv::waitKey(10);
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}