#!/usr/bin/env python3

import rospy
import math
import time
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.msg import ProximitySensors  # /proximity_sensors
from ros_basics_msgs.srv import *
# dynamic_reconfigure for tuning
from dynamic_reconfigure.server import Server
from ros_basics_exercise.cfg import ThymioControllerConfig
from ros_basics_msgs.srv import SetWaypoints
from geometry_msgs.msg import Pose2D

# ThymioController Class
class ThymioController:
    def __init__(self):
        """
        initialize variables and parameters, include three aspects as follows:
        1) robot state variables and PID control parameters,
        2) sensor variables and obstacle avoidance parameter,
        3) ros message objects & ros node/subscriber/publisher/serviceproxy setup.
        """
        # I. Robot State and PID Control
        self.reached = True  # bool, robot state
        # robot pose
        self.x = 0.  # float
        self.y = 0.  # float
        self.yaw = 0.  # float
        # current state error
        self.yaw_err = 0.
        self.pos_err = 0.
        # initial state error
        self.int_yaw = 0.
        self.int_pos = 0.
        # previous state error
        self.prev_yaw_err = 0.
        self.prev_pos_err = 0.
        # max limit
        self.w_max = pi  # rad/s
        self.v_max = 0.05  # m/s
        # PID parameters
        self.w_pid = [3, 0, 0]
        self.v_pid = [2, 0, 0]

        # II. Sensing and Obstacle Avoidance
        # sensor data
        self.prox_h0 = 0.  # float
        self.prox_h1 = 0.  # float
        self.prox_h2 = 0.  # float
        self.prox_h3 = 0.  # float
        self.prox_h4 = 0.  # float

        # obstacle avoidance parameters
        self.lf = 0  # int, left wall following turning counter
        self.oa = False  # bool obstacle avoidance mode
        self.oa_forward = False  # bool
        self.oa_turn = False  # bool
        self.oa_obstacle_ahead = False  # bool
        self.oa_w = pi/4  # float angular speed in obstacle avoidance
        self.oa_w_t = 0.5  # float turning time in obstacle avoidance
        self.oa_v = 0.05  # float linear speed in obstacle avoidance
        self.oa_v_t = 0.8  # float forwarding time in obstacle avoidance
        # sensor basic threshold
        # float [m] threshold detecting obstacle for sensor 1 2 3
        self.oa_th_middle = 0.03
        # float [m] threshold detecting obstacle for sensor 0 4
        self.oa_th_side = 0.025

        # III. ROS Node, Topics & Service
        # message objects
        self.pose = SimplePoseStamped()
        self.proximity = ProximitySensors()
        self.simple_vel = SimpleVelocities()
        # bool, verbose flag for check_waypoint_reached service
        self.verbose = False

        # Init node
        rospy.init_node('thymio_control_pnode', anonymous=True)
        rospy.loginfo('Node "thymio_control_pnode" created')

        # Create service proxies
        rospy.loginfo('Wait for current_waypoint_srv')
        rospy.wait_for_service('current_waypoint')
        self.current_waypoint_srv = rospy.ServiceProxy(
            'current_waypoint', CurrentWaypoint)

        rospy.loginfo('Wait for check_waypoint_reached_srv')
        rospy.wait_for_service('check_waypoint_reached')
        self.check_waypoint_reached_srv = rospy.ServiceProxy(
            'check_waypoint_reached', CheckWaypointReached)

        rospy.loginfo('Wait for get_waypoints_srv')
        rospy.wait_for_service('get_waypoints')
        self.get_waypoints_srv = rospy.ServiceProxy(
            'get_waypoints', GetWaypoints)

        # Create subscribers
        rospy.loginfo('Create robot_pose_sub')  # average rate: 10.00
        self.robot_pose_sub = rospy.Subscriber(
            'robot_pose', SimplePoseStamped, self.robot_pose_cb)
        rospy.loginfo('Create proximity_sub')  # average rate: 2.500
        self.proximity_sub = rospy.Subscriber(
            'proximity_sensors', ProximitySensors, self.proximity_cb)

        # Create publisher
        rospy.loginfo('Create set_velocities_pub')
        self.set_velocities_pub = rospy.Publisher(
            'set_velocities', SimpleVelocities, queue_size=10)

        # Create dynamic_reconfigure service
        rospy.loginfo('Dynamic parameter reconfigure')
        srv = Server(ThymioControllerConfig, self.reconfig)
        # parameters for dynamic_reconfigure
        self.test_goal_added = False  # bool
        self.dynamic_param = {
            'mode': 0,
            'W_MAX': self.w_max, 'V_MAX': self.v_max,
            'W_KP': self.w_pid[0], 'W_KI': self.w_pid[1], 'W_KD': self.w_pid[2],
            'V_KP': self.v_pid[0], 'V_KI': self.v_pid[1], 'V_KD': self.v_pid[2],
            'test_x': 0.0, 'test_y': 0.0, 'test_yaw': 0.0, 'start_test': False
        }

    def reconfig(self, config, level):
        """reconfigure function for dynamic_reconfigure"""
        self.dynamic_param = config
        self.w_max = self.dynamic_param['W_MAX']
        self.v_max = self.dynamic_param['V_MAX']
        self.w_pid = [self.dynamic_param['W_KP'],
            self.dynamic_param['W_KI'], self.dynamic_param['W_KD']]
        self.v_pid = [self.dynamic_param['V_KP'],
            self.dynamic_param['V_KI'], self.dynamic_param['V_KD']]
        print("W_MAX = {}, V_MAX = {}".format(self.w_max, self.v_max))
        print("w_pid = [{}, {}, {}]\nv_pid = [{}, {}, {}]".format(
            self.dynamic_param['W_KP'], self.dynamic_param['W_KI'], self.dynamic_param['W_KD'],
            self.dynamic_param['V_KP'], self.dynamic_param['V_KI'], self.dynamic_param['V_KD']))
        if self.dynamic_param.get('mode') == 0:
            mode = "Obstacle Avoidance"
        elif self.dynamic_param.get('mode') == 1:
            mode = "Test"
            print("goal for test: [{}, {}]".format(self.dynamic_param['test_x'], self.dynamic_param['test_y']))
        print("Mode: ", mode)
        return config

    @staticmethod
    def reg_ang(angle):
        """regulate the angle between -pi to pi"""
        if angle > pi:
            angle -= 2*pi
        elif angle < -pi:
            angle += 2*pi
        return angle

    @staticmethod
    def pid_control(err, prev_err, integrator, pid, u_max):
        """pid control and integrator anti-windup"""
        Ts = 1/10  # 10Hz
        integrator = integrator + Ts*(err+prev_err)/2
        derivative = (err-prev_err)/Ts
        u = pid[0]*err + pid[1]*integrator + pid[2]*derivative
        if u > u_max:
            if pid[1] != 0:
                integrator = integrator - Ts*(err+prev_err)/2
            u = u_max
        elif u < -u_max:
            if pid[1] != 0:
                integrator = integrator - Ts*(err+prev_err)/2
            u = -u_max
        return integrator, u

    # Create service requests
    def request_current_waypoint(self):
        """request current_waypoint service to check whether waypoint list is empty and return next waypoint (goal)"""
        resp = self.current_waypoint_srv()
        return [resp.goal, resp.is_empty]

    def request_check_waypoint_reached(self):
        """request check_waypoint_reached service to check whether the robot reach the next waypoint"""
        resp = self.check_waypoint_reached_srv(self.pose, self.verbose)
        return resp.reached

    def request_get_waypoints(self):
        """request get_waypoints service to get targeted waypoints"""
        resp = self.get_waypoints_srv()
        return resp.waypoints

    def robot_pose_cb(self, simple_pose_stamped):
        """subscribe to the robot_pose topic to get the robot's current pose"""
        self.pose = simple_pose_stamped
        self.x = simple_pose_stamped.pose.xyz.x
        self.y = simple_pose_stamped.pose.xyz.y
        self.yaw = simple_pose_stamped.pose.rpy.yaw
        # rospy.loginfo("Current robot pose: x: {}, y: {}, yaw: {} ".format(self.x, self.y, self.yaw))

    def proximity_cb(self, proximity_sensors):
        """subscribe to the proximity_sensors topic to get the values of proximity sensors"""
        self.proximity = proximity_sensors
        self.prox_h0 = proximity_sensors.values[0]
        self.prox_h1 = proximity_sensors.values[1]
        self.prox_h2 = proximity_sensors.values[2]
        self.prox_h3 = proximity_sensors.values[3]
        self.prox_h4 = proximity_sensors.values[4]
        # rospy.loginfo("sensor0: {}, sensor1: {}, sensor2: {}, sensor3: {}, sensor4: {}, ".format(self.prox_h0, self.prox_h1, self.prox_h2, self.prox_h3, self.prox_h4))

    def stop(self):
        """set robot state when in case of empty waypoint list or arrival"""
        self.simple_vel.v = 0.
        self.simple_vel.w = 0.
        self.int_yaw = 0.
        self.int_pos = 0.
        self.prev_yaw_err = 0.
        self.prev_pos_err = 0.
        self.yaw_err = 0.
        self.pos_err = 0.

    def run_with_obstacle_avoidance(self):
        """Thymio operates with obstacle_avoidance"""
        loop_rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            goal, is_empty = self.request_current_waypoint()
            if is_empty:
                self.stop()
                # rospy.loginfo("Waypoint list is empty")
            else:
                self.reached = self.request_check_waypoint_reached()
                if self.reached:
                    self.stop()
                    self.oa = False
                    rospy.loginfo("Waypoint reached")
                else:
                    if not self.oa:
                        tar_yaw = math.atan2(
                            goal.x - self.x, -(goal.y - self.y))
                        self.yaw_err = self.reg_ang(tar_yaw - self.yaw)
                        self.pos_err = math.sqrt(
                            (goal.x - self.x)**2+(goal.y - self.y)**2)
                        self.int_yaw, self.simple_vel.w = self.pid_control(
                            self.yaw_err, self.prev_yaw_err, self.int_yaw, self.w_pid, self.w_max)
                        self.simple_vel.v = self.simple_vel.v / \
                            (abs(self.yaw_err)/(pi/90))
                        # rospy.loginfo("Start wandering")
                        if((min(self.prox_h1, self.prox_h2, self.prox_h3) < self.oa_th_middle or min(self.prox_h4, self.prox_h0) < self.oa_th_side) and abs(self.yaw_err) < pi/90):
                            rospy.loginfo("Entering Obstacle Avoidance")
                            self.oa = True
                            self.lf = 0
                            self.stop()
                        else:
                            self.int_pos, self.simple_vel.v = self.pid_control(
                                self.pos_err, self.prev_pos_err, self.int_pos, self.v_pid, self.v_max)
                            if abs(self.yaw_err) > pi/45:
                                self.simple_vel.v = self.simple_vel.v / \
                                    (abs(self.yaw_err)/(pi/90))
                    else:
                        self.oa_obstacle_ahead = (min(self.prox_h1, self.prox_h2, self.prox_h3) < self.oa_th_middle or min(
                            self.prox_h0, self.prox_h4) < self.oa_th_side)
                        if (not self.oa_forward):
                            if (self.oa_obstacle_ahead):
                                rospy.loginfo("Try turning left")
                                self.simple_vel.v = 0
                                self.simple_vel.w = self.oa_w
                                self.set_velocities_pub.publish(
                                    self.simple_vel)
                                self.lf += 1
                                time.sleep(self.oa_w_t)
                            elif (not self.oa_turn and not self.oa_obstacle_ahead):
                                rospy.loginfo(
                                    "Turned to direction with no obstacles. try going forward")
                                self.oa_forward = True
                        if (self.oa_forward):
                            rospy.loginfo(
                                "Going forward in Obstacle Avoidance")
                            self.simple_vel.v = 0
                            self.simple_vel.w = self.oa_w
                            self.set_velocities_pub.publish(self.simple_vel)
                            time.sleep(self.oa_w_t)
                            self.simple_vel.v = self.oa_v
                            self.simple_vel.w = 0
                            self.sleep(self.oa_v_t)
                            self.simple_vel.v = 0
                            self.simple_vel.w = -self.oa_w
                            self.set_velocities_pub.publish(self.simple_vel)
                            time.sleep(self.oa_w_t)
                            self.oa_forward = False
                            self.oa_turn = True
                        if (self.oa_turn):
                            if (not self.oa_obstacle_ahead):
                                rospy.loginfo(
                                    "No obstacles ahead, try turning right")
                                self.simple_vel.v = 0
                            rospy.loginfo("Exiting Obstacle Avoidance")
                    # update error
                    self.prev_yaw_err, self.prev_pos_err = self.yaw_err, self.pos_err
            self.set_velocities_pub.publish(self.simple_vel)
            loop_rate.sleep()

    def test_mode(self):
        if not self.test_goal_added:
            p = Pose2D()
            p.x = self.dynamic_param['test_x']
            p.y = self.dynamic_param['test_y']
            p.theta = self.dynamic_param['test_yaw']
            wpt = []
            wpt.append(p)
            rospy.wait_for_service('set_waypoints')
            try:
                set_wpt = rospy.ServiceProxy('set_waypoints', SetWaypoints)
                resp = set_wpt(wpt)
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
            self.test_goal_added = True
        start_test = self.dynamic_param.get('start_test')
        goal, is_empty = self.request_current_waypoint()
        if not start_test:
            self.stop()
        else:
            if not is_empty:
                self.reached = self.request_check_waypoint_reached()
                if not self.reached:
                    tar_yaw = math.atan2(goal.x - self.x, -(goal.y - self.y))
                    self.yaw_err = self.reg_ang(tar_yaw - self.yaw)
                    self.pos_err = math.sqrt(
                        (goal.x - self.x)**2+(goal.y - self.y)**2)
                    self.int_yaw, self.simple_vel.w = self.pid_control(
                        self.yaw_err, self.prev_yaw_err, self.int_yaw, self.w_pid, self.w_max)
                    self.simple_vel.v = self.simple_vel.v / \
                        (abs(self.yaw_err)/(pi/90))
                    self.int_pos, self.simple_vel.v = self.pid_control(
                        self.pos_err, self.prev_pos_err, self.int_pos, self.v_pid, self.v_max)
                    if abs(self.yaw_err) > pi/45:
                        self.simple_vel.v = self.simple_vel.v / \
                            (abs(self.yaw_err)/(pi/90))
                    self.prev_yaw_err, self.prev_pos_err = self.yaw_err, self.pos_err
                else:
                    # !cancel regulate to targeted angle due to the unbalanced model
                    # tar_yaw = self.dynamic_param['test_yaw']
                    # while self.reg_ang(tar_yaw - self.yaw) > 0.015:
                    #     self.simple_vel.v = 0
                    #     self.simple_vel.w = 0.5
                    #     self.set_velocities_pub.publish(self.simple_vel)
                    #     time.sleep(0.1)
                    self.stop()
                    self.test_goal_added = False
            else:
                self.stop()
                self.test_goal_added = False


    def obstacle_avoidance_mode(self):
        goal, is_empty = self.request_current_waypoint()
        if is_empty:
            self.stop()
            # rospy.loginfo("Waypoint list is empty")
        else:
            self.reached = self.request_check_waypoint_reached()
            if self.reached:
                self.stop()
                self.oa = False
                rospy.loginfo("Waypoint reached")
            else:
                if not self.oa:
                    tar_yaw = math.atan2(goal.x - self.x, -(goal.y - self.y))
                    self.yaw_err = self.reg_ang(tar_yaw - self.yaw)
                    self.pos_err = math.sqrt(
                        (goal.x - self.x)**2+(goal.y - self.y)**2)
                    self.int_yaw, self.simple_vel.w = self.pid_control(
                        self.yaw_err, self.prev_yaw_err, self.int_yaw, self.w_pid, self.w_max)
                    self.simple_vel.v = self.simple_vel.v / \
                        (abs(self.yaw_err)/(pi/90))
                    if((min(self.prox_h1, self.prox_h2, self.prox_h3) < self.oa_th_middle or min(self.prox_h4, self.prox_h0) < self.oa_th_side) and abs(self.yaw_err) < pi/90):
                        rospy.loginfo("Entering Obstacle Avoidance")
                        self.oa = True
                        self.lf = 0
                        self.stop()
                    else:
                        self.int_pos, self.simple_vel.v = self.pid_control(
                            self.pos_err, self.prev_pos_err, self.int_pos, self.v_pid, self.v_max)
                        if abs(self.yaw_err) > pi/45:
                            self.simple_vel.v = self.simple_vel.v / \
                                (abs(self.yaw_err)/(pi/90))
                else:
                    self.oa_obstacle_ahead = (min(self.prox_h1, self.prox_h2, self.prox_h3) < self.oa_th_middle or min(
                        self.prox_h0, self.prox_h4) < self.oa_th_side)
                    if (not self.oa_forward):
                        if (self.oa_obstacle_ahead):
                            rospy.loginfo("Try turning left")
                            self.simple_vel.v = 0
                            self.simple_vel.w = self.oa_w
                            self.set_velocities_pub.publish(self.simple_vel)
                            self.lf += 1
                            time.sleep(self.oa_w_t)
                        elif (not self.oa_turn and not self.oa_obstacle_ahead):
                            rospy.loginfo(
                                "Turned to direction with no obstacles. try going forward")
                            self.oa_forward = True
                    if (self.oa_forward):
                        rospy.loginfo("Going forward in Obstacle Avoidance")
                        self.simple_vel.v = 0
                        self.simple_vel.w = self.oa_w
                        self.set_velocities_pub.publish(self.simple_vel)
                        time.sleep(self.oa_w_t)
                        self.simple_vel.v = self.oa_v
                        self.simple_vel.w = 0
                        self.set_velocities_pub.publish(self.simple_vel)
                        time.sleep(self.oa_v_t)
                        self.simple_vel.v = 0
                        self.simple_vel.w = -self.oa_w
                        self.set_velocities_pub.publish(self.simple_vel)
                        time.sleep(self.oa_w_t)
                        self.oa_forward = False
                        self.oa_turn = True
                    if (self.oa_turn):
                        if (not self.oa_obstacle_ahead):
                            rospy.loginfo(
                                "No obstacles ahead, try turning right")
                            self.simple_vel.v = 0
                            self.simple_vel.w = -self.oa_w
                            self.set_velocities_pub.publish(self.simple_vel)
                            self.lf -= 1
                            time.sleep(self.oa_w_t)
                        else:
                            rospy.loginfo("Detected obstacles, continue to left wall following")
                            self.oa_turn = False
                    if (self.lf == 0):
                        self.oa = False
                        rospy.loginfo("Exiting Obstacle Avoidance")
                # update error
                self.prev_yaw_err, self.prev_pos_err = self.yaw_err, self.pos_err

    def run(self):
        loop_rate = rospy.Rate(10)  # Hz
        # /set_waypoints
        while not rospy.is_shutdown():
            mode = self.dynamic_param.get('mode')
            if mode == 0:
                self.obstacle_avoidance_mode()
            elif mode == 1:
                self.test_mode()
            else:
                rospy.logerr('Unsupported mode!')
            self.set_velocities_pub.publish(self.simple_vel)
            loop_rate.sleep()

if __name__ == '__main__':
    pi = math.pi
    tc = ThymioController()
    try:
        # run directly tc.run_with_obstacle_avoidance()
        # run with different modes tc.run()
        tc.run()
    except rospy.ROSInterruptException:
        pass
