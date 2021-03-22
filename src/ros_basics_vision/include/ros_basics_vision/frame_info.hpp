#ifndef ROS_TP_FRAME_INFO_HPP
#define ROS_TP_FRAME_INFO_HPP

#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_msgs/GetWaypoints.h>
#include <ros_basics_msgs/AddWaypoint.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace ros_tp {
    class FrameInfo {
    public:
        FrameInfo(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<image_transport::ImageTransport> it, double pix2mm, int width, int height) : _nh(nh), _it(it), _pix2mm(pix2mm)
        {
            _annot_image_pub = _it->advertise("robot_view/image_annot", 1);
            _get_waypoints_cli = _nh->serviceClient<ros_basics_msgs::GetWaypoints>("get_waypoints");
            _get_waypoints_cli.waitForExistence();
            _prev_stamp = ros::Time::now();

            cv::Vec3d rvec(0., 0., 0.);
            cv::Vec3d tvec((width / 2.) * _pix2mm / 1000, (height / 2.) * _pix2mm / 1000, -0.63);
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat C = R.inv() * tvec;
            _ccenter = cv::Point(C.at<double>(0, 0) * 1000. / _pix2mm, C.at<double>(0, 1) * 1000. / _pix2mm);
        }

        FrameInfo(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<ArucoDetector> ad, std::shared_ptr<image_transport::ImageTransport> it, double pix2mm, int width, int height) : _nh(nh), _ad(ad), _it(it), _pix2mm(pix2mm)
        {
            _annot_image_pub = _it->advertise("robot_view/image_annot", 1);
            _get_waypoints_cli = _nh->serviceClient<ros_basics_msgs::GetWaypoints>("get_waypoints");
            _get_waypoints_cli.waitForExistence();
            _prev_stamp = ros::Time::now();

            cv::Vec3d rvec(0., 0., 0.);
            cv::Vec3d tvec((width / 2.) * _pix2mm / 1000, (height / 2.) * _pix2mm / 1000, -0.63);
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat C = R.inv() * tvec;
            _ccenter = cv::Point(C.at<double>(0, 0) * 1000. / _pix2mm, C.at<double>(0, 1) * 1000. / _pix2mm);
        }

        void draw_info(cv::Mat& frame)
        {
            ros::Time now = ros::Time::now();
            double dt = ros::Duration(now - _prev_stamp).toSec();

            _ad->annotate_image(frame); // publish annotated image with aruco marker
            draw_pose(frame);
            draw_waypoints(frame);
            // draw_origin(frame);
            draw_camera_origin(frame);
            draw_axes(frame);
            draw_fps(frame, dt);

            _prev_stamp = now;
            publish(frame);
        }

        void draw_pose(cv::Mat& frame, cv::Point2f pos, double angle)
        {
            std::stringstream yaw_stream;
            yaw_stream << std::fixed << std::setprecision(2) << angle * 180. / M_PI;
            cv::putText(frame,
                "Hdg: " + yaw_stream.str(),
                cv::Point(pos.x, pos.y - 30),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(255, 200, 15), // CV_RGB(180, 0, 0)CV_RGB(255, 200, 15), // CV_RGB(180, 0, 0)
                2);

            std::stringstream pos_stream;
            pos_stream << std::fixed << std::setprecision(4) << (pos.x - _ccenter.x) * _pix2mm / 1000. << ", " << -(pos.y - _ccenter.y) * _pix2mm / 1000.;
            cv::putText(frame,
                "Pos: (" + pos_stream.str() + ")",
                cv::Point(pos.x, pos.y),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(255, 200, 15), // CV_RGB(180, 0, 0)
                2);
        }

        void draw_pose(cv::Mat& frame)
        {
            if (_ad->get_current_poses().size()) {
                // heading on frame
                cv::Point2f centroid = _ad->get_pixel_positions()[0];
                cv::Vec6d pose = _ad->get_current_poses()[0];
                if (_ad->get_current_poses().size()) {
                    draw_pose(frame, centroid, pose[5]);
                }
            }
        }

        void draw_waypoints(cv::Mat& frame)
        {
            ros_basics_msgs::GetWaypoints srv;
            if (_get_waypoints_cli.call(srv)) {
                auto waypoints = srv.response.waypoints;
                for (size_t i = 0; i < waypoints.size(); ++i) {
                    cv::Point wp((waypoints[i].x * 1000.) / _pix2mm + _ccenter.x, (waypoints[i].y * 1000.) / _pix2mm + _ccenter.y);
                    wp.y = 2 * _ccenter.y - wp.y;
                    cv::circle(frame, wp, 5, cv::Scalar(255, 255, 0), cv::FILLED, 10, 0);
                    std::stringstream stream;
                    stream << i;
                    cv::putText(frame,
                        stream.str(),
                        wp + cv::Point(10, -10),
                        cv::FONT_HERSHEY_DUPLEX,
                        0.5,
                        cv::Scalar(255, 255, 0),
                        2);
                }
            }
            else {
                ROS_ERROR("Failed to call service get_waypoints");
            }
        }

        void draw_origin(cv::Mat& frame)
        {
            cv::putText(frame,
                "O(0, 0)",
                cv::Point(frame.size().width / 2 + 10, frame.size().height / 2 - 10),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(255, 0, 0),
                2);
            cv::circle(frame, cv::Point(frame.size().width / 2, frame.size().height / 2), 4, cv::Scalar(0, 0, 255), cv::FILLED, 10, 0);
        }

        void draw_camera_origin(cv::Mat& frame)
        {
            cv::circle(frame, _ccenter, 4, cv::Scalar(0, 0, 255), cv::FILLED, 10, 0);
            cv::putText(frame,
                "C(0, 0)",
                _ccenter + cv::Point(10, -10),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(255, 0, 0),
                2);
        }

        void draw_axes(cv::Mat& frame)
        {
            cv::arrowedLine(frame, _ccenter, _ccenter - cv::Point(0, frame.size().height * 0.1), CV_RGB(255, 0, 0), 2, 8, 0, 0.1);
            cv::arrowedLine(frame, _ccenter, _ccenter + cv::Point(frame.size().height * 0.1, 0), CV_RGB(255, 0, 0), 2, 8, 0, 0.1);

            cv::putText(frame,
                "y",
                _ccenter - cv::Point(30, frame.size().height * 0.1) / 2.5,
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(255, 0, 0),
                1);
            cv::putText(frame,
                "x",
                _ccenter + cv::Point(frame.size().height * 0.1, 30) / 2.5,
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(255, 0, 0),
                1);
        }

        void draw_fps(cv::Mat& frame, double dt)
        {
            std::stringstream stream;
            stream << std::fixed << std::setprecision(1) << 1 / dt;
            cv::putText(frame,
                "FPS: " + stream.str(),
                cv::Point(frame.size().width * 0.03, frame.size().height * 0.05),
                cv::FONT_HERSHEY_DUPLEX,
                0.5,
                CV_RGB(180, 0, 0),
                2);
        }

        void publish(cv::Mat& frame)
        {
            sensor_msgs::ImagePtr annot_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            _annot_image_pub.publish(annot_msg);
        }

    protected:
        image_transport::Publisher _annot_image_pub;
        std::shared_ptr<ros::NodeHandle> _nh;
        std::shared_ptr<ArucoDetector> _ad;
        std::shared_ptr<image_transport::ImageTransport> _it;
        double _pix2mm;
        ros::ServiceClient _get_waypoints_cli;
        ros::Time _prev_stamp;
        ros::Duration _dt;
        cv::Point _ccenter;
    };

} // namespace ros_tp

#endif