#ifndef ROS_TP_CONTROLLER_BASE_H
#define ROS_TP_CONTROLLER_BASE_H

#include <ros_basics_msgs/SimplePoseStamped.h>
#include <ros_basics_msgs/AddWaypoint.h>
#include <ros_basics_msgs/RemoveWaypoint.h>
#include <ros_basics_msgs/CurrentWaypoint.h>
#include <ros_basics_msgs/GetWaypoints.h>
#include <ros_basics_msgs/SetWaypoints.h>
#include <ros_basics_msgs/CheckWaypointReached.h>

#include <geometry_msgs/Pose2D.h>

#include <cassert>
#include <chrono>
#include <mutex>

namespace ros_tp {
    namespace defaults {
        struct ControllerParams {
            static constexpr double precision = 0.015;
        };
    } // namespace defaults

    template <typename Params>
    class ControllerBase {
    public:
        using Pose = ros_basics_msgs::SimplePoseStamped;

        ControllerBase(std::shared_ptr<ros::NodeHandle> nh) : _nh(nh)
        {
            _add_waypoint_srv = _nh->advertiseService("add_waypoint", &ControllerBase::_add_waypoint_service_cb, this);
            _remove_waypoint_srv = _nh->advertiseService("remove_waypoint", &ControllerBase::_remove_waypoint_service_cb, this);
            _current_waypoint_srv = _nh->advertiseService("current_waypoint", &ControllerBase::_current_waypoint_service_cb, this);
            _get_waypoints_srv = _nh->advertiseService("get_waypoints", &ControllerBase::_get_waypoints_service_cb, this);
            _set_waypoints_srv = _nh->advertiseService("set_waypoints", &ControllerBase::_set_waypoints_service_cb, this);
            _check_waypoint_reached_srv = _nh->advertiseService("check_waypoint_reached", &ControllerBase::_check_waypoint_reached_service_cb, this);
        }

        template <typename ThymioInterface>
        void move(const Pose& pose, ThymioInterface& ti, bool verbose = false)
        {
            ROS_DEBUG_ONCE("You are using the controller's server API");
        }

        bool check_current_waypoint(const Pose& pose, bool verbose = false)
        {
            const std::lock_guard<std::mutex> lock(_waypoint_lock);
            if (_waypoints.size()) {
                double distance_to_goal = sqrt(pow(pose.pose.xyz.x - _waypoints[0].x, 2.) + pow(pose.pose.xyz.y - _waypoints[0].y, 2.));
                if (distance_to_goal <= Params::ControllerParams::precision) {
                    if (verbose) {
                        ROS_INFO("Waypoint (%f, %f) reached at a distance of %f m", _waypoints[0].x, _waypoints[0].y, distance_to_goal);
                    }
                    _waypoints.erase(_waypoints.begin());
                    return true;
                }
                else if (verbose) {
                    ROS_INFO("Tracking waypoint (%f, %f) -> Current distance is %f m", _waypoints[0].x, _waypoints[0].y, distance_to_goal);
                    return false;
                }
            }
            return false;
        }

        bool add_waypoint(const geometry_msgs::Pose2D& waypoint)
        {
            const std::lock_guard<std::mutex> lock(_waypoint_lock);
            _waypoints.push_back(waypoint);
            return true;
        }

        bool remove_waypoint(size_t idx)
        {
            const std::lock_guard<std::mutex> lock(_waypoint_lock);
            if (_waypoints.size() > idx) {
                _waypoints.erase(_waypoints.begin() + idx);
                return true;
            }
            else {
                return false;
            }
        }

        std::vector<geometry_msgs::Pose2D> get_waypoints()
        {
            const std::lock_guard<std::mutex> lock(_waypoint_lock);
            return _waypoints;
        }

        void set_waypoints(const std::vector<geometry_msgs::Pose2D>& new_waypoints)
        {
            const std::lock_guard<std::mutex> lock(_waypoint_lock);
            _waypoints.clear();
            std::copy(new_waypoints.begin(), new_waypoints.end(), std::back_inserter(_waypoints));
        }

        std::pair<geometry_msgs::Pose2D, bool> current_waypoint()
        {
            const std::lock_guard<std::mutex> lock(_waypoint_lock);
            if (_waypoints.size() == 0) {
                return std::make_pair(geometry_msgs::Pose2D(), true);
            }
            else {
                return std::make_pair(_waypoints[0], false);
            }
        }

    protected:
        bool _add_waypoint_service_cb(
            ros_basics_msgs::AddWaypoint::Request& req,
            ros_basics_msgs::AddWaypoint::Response& res)
        {
            add_waypoint(req.goal);
            return true;
        }

        bool _remove_waypoint_service_cb(
            ros_basics_msgs::RemoveWaypoint::Request& req,
            ros_basics_msgs::RemoveWaypoint::Response& res)
        {
            res.removed = remove_waypoint(req.idx);
            return true;
        }

        bool _current_waypoint_service_cb(
            ros_basics_msgs::CurrentWaypoint::Request& req,
            ros_basics_msgs::CurrentWaypoint::Response& res)
        {
            std::tie(res.goal, res.is_empty) = current_waypoint();
            return true;
        }

        bool _get_waypoints_service_cb(
            ros_basics_msgs::GetWaypoints::Request& req,
            ros_basics_msgs::GetWaypoints::Response& res)
        {
            res.waypoints = get_waypoints();
            return true;
        }

        bool _set_waypoints_service_cb(
            ros_basics_msgs::SetWaypoints::Request& req,
            ros_basics_msgs::SetWaypoints::Response& res)
        {
            set_waypoints(req.waypoints);
            return true;
        }

        bool _check_waypoint_reached_service_cb(
            ros_basics_msgs::CheckWaypointReached::Request& req,
            ros_basics_msgs::CheckWaypointReached::Response& res)
        {
            res.reached = check_current_waypoint(req.pose, req.verbose);
            return true;
        }

        double _angle_to_pipi(double angle)
        {
            while (true) {
                if (angle < -M_PI) {
                    angle += 2. * M_PI;
                }
                if (angle > M_PI) {
                    angle -= 2. * M_PI;
                }
                if (abs(angle) <= M_PI) {
                    break;
                }
            }
            return angle;
        }

        std::shared_ptr<ros::NodeHandle> _nh;
        ros::ServiceServer _add_waypoint_srv;
        ros::ServiceServer _remove_waypoint_srv;
        ros::ServiceServer _current_waypoint_srv;
        ros::ServiceServer _get_waypoints_srv;
        ros::ServiceServer _set_waypoints_srv;
        ros::ServiceServer _check_waypoint_reached_srv;

        std::vector<geometry_msgs::Pose2D> _waypoints;
        std::mutex _waypoint_lock;
    };

} // namespace ros_tp

#endif