#!/usr/bin/env python3

import rospy
import math
import time
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.msg import ProximitySensors  # /proximity_sensors
from ros_basics_msgs.srv import *

# ThymioController Class
class ThymioController:
    def __init__(self):
        """
        initialize variables and parameters, include three aspects as follows:
        1) robot state variables and PID control parameters,
        2) sensor variables and obstacle avoidance parameter,
        3) ros message objects & ros node/subscriber/publisher/serviceproxy setup.
        """
        ## I. Robot State and PID Control
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
        self.w_pid = [3.6, 0, 0] # simu [3, 0, 0], test:  [3.6, 0, 0]
        self.v_pid = [2, 0, 0]

        ## II. Sensing and Obstacle Avoidance
        # sensor data
        self.prox_h0 = 0. # float
        self.prox_h1 = 0. # float
        self.prox_h2 = 0. # float
        self.prox_h3 = 0. # float
        self.prox_h4 = 0.  # float

        # obstacle avoidance parameters
        self.lf = 0 # int, left wall following turning counter
        self.oa = False  # bool obstacle avoidance mode
        self.oa_forward= False # bool
        self.oa_turn= False # ool
        self.oa_obstacle_ahead = False # bool
        self.oa_turn_more = False # bool
        # sensor basic threshold
        self.offset = 300 # int, tuning offset for real-world test
        self.s0 = 3420  - self.offset #float sensor0 basic value at 2cm
        self.s1 = 3400  - self.offset #float sensor1 basic value at 2cm
        self.s2 = 3480  - self.offset #float sensor2 basic value at 2cm
        self.s3 = 3460  - self.offset #float sensor3 basic value at 2cm
        self.s4 = 3020  - self.offset #float sensor4 basic value at 2cm


        ## III. ROS Node, Topics & Service
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
        Ts = 0.1  # 10Hz
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
        #rospy.loginfo("sensor0: {}, sensor1: {}, sensor2: {}, sensor3: {}, sensor4: {}, ".format(self.prox_h0, self.prox_h1, self.prox_h2, self.prox_h3, self.prox_h4))

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

    def stop_read(self):
        """stop Thymio to read proximity sensor value for real-world deployment"""
        loop_rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            rospy.loginfo("sensor0: {}, sensor1: {}, sensor2: {}, sensor3: {}, sensor4: {}, ".format(
               self.prox_h0, self.prox_h1, self.prox_h2, self.prox_h3, self.prox_h4))
            goal, is_empty = self.request_current_waypoint()
            if not is_empty:
                tar_yaw = math.atan2( (goal.y - self.y),goal.x - self.x)
                print("yaw:",tar_yaw)
            self.stop()
            self.set_velocities_pub.publish(self.simple_vel)
            loop_rate.sleep()

    def calibrate_rot(self):
        """set rotation command to Thymio to calibrate the w_pid """
        loop_rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            self.simple_vel.v = 0
            self.simple_vel.w = pi/2
            self.set_velocities_pub.publish(self.simple_vel)
            time.sleep(1)
            self.simple_vel.v = 0
            self.simple_vel.w = 0
            self.set_velocities_pub.publish(self.simple_vel)
            time.sleep(3)
            loop_rate.sleep()

    def run_with_obstacle_avoidance(self):
        """Thymio operates with obstacle_avoidance"""
        loop_rate = rospy.Rate(10)  # Hz
        while not rospy.is_shutdown():
            goal, is_empty = self.request_current_waypoint()
            if is_empty:
                self.stop()
                #rospy.loginfo("Waypoint list is empty")
            else:
                self.reached = self.request_check_waypoint_reached()
                if self.reached:
                    self.stop()
                    self.oa = False
                    rospy.loginfo("Waypoint reached")
                else:
                    if not self.oa:
                        tar_yaw = math.atan2(goal.y - self.y,goal.x - self.x)
                        self.yaw_err = self.reg_ang(tar_yaw - self.yaw)
                        self.pos_err = math.sqrt((goal.x - self.x)**2+(goal.y - self.y)**2)
                        self.int_yaw, self.simple_vel.w = self.pid_control(self.yaw_err, self.prev_yaw_err, self.int_yaw, self.w_pid, self.w_max)
                        self.simple_vel.v = self.simple_vel.v / \
                            (abs(self.yaw_err)/(pi/90))
                        #rospy.loginfo("Start wandering")
                        if(max(self.prox_h1-self.s1,self.prox_h2-self.s2,self.prox_h3-self.s1) > 0 and abs(self.yaw_err) < pi/45):
                            rospy.loginfo("Entering Obstacle Avoidance")
                            self.oa = True
                            self.oa_turn_more = False
                            self.lf = 0
                            self.stop()
                        else:
                            self.int_pos, self.simple_vel.v = self.pid_control(self.pos_err, self.prev_pos_err, self.int_pos, self.v_pid, self.v_max)
                            if abs(self.yaw_err) > pi/45:
                                self.simple_vel.v = self.simple_vel.v / \
                                    (abs(self.yaw_err)/(pi/90))
                    else:
                        self.oa_obstacle_ahead = (max(self.prox_h1-self.s1,self.prox_h2-self.s2,self.prox_h3-self.s3,self.prox_h4-self.s4) > 0)
                        if (not self.oa_forward):
                            if (self.oa_obstacle_ahead or (self.prox_h0 - self.s0 > 0)):
                                rospy.loginfo("Try turning left")
                                self.simple_vel.v = 0
                                self.simple_vel.w = 1.5
                                self.set_velocities_pub.publish(self.simple_vel)
                                self.lf += 1
                                if not self.oa_turn_more: #turn left a small angle to explore if no obstacle ahead in previous step
                                    time.sleep(0.5) #try turning left for first time
                                else: #turn left a big angle to avoid if obstacle ahead in previous step
                                    time.sleep(1)
                            elif (not self.oa_turn and not self.oa_obstacle_ahead):
                                rospy.loginfo("Turned to direction with no obstacles. try going forward")
                                self.oa_turn_more = False
                                self.oa_forward = True
                            #self.oa_obstacle_ahead and
                        if (self.oa_forward):
                            rospy.loginfo("Going forward in Obstacle Avoidance")
                            self.simple_vel.v = 0.05 # 0.08
                            self.simple_vel.w = 0
                            self.set_velocities_pub.publish(self.simple_vel)
                            time.sleep(0.8)
                            self.oa_forward = False
                            self.oa_turn= True #bool
                        if (self.oa_turn):
                            if (not self.oa_obstacle_ahead and not (self.prox_h4-self.s4 > 0)):
                                rospy.loginfo("No obstacles ahead, try turning right")
                                self.simple_vel.v = 0
                                self.simple_vel.w = -1.5
                                self.set_velocities_pub.publish(self.simple_vel)
                                self.lf -= 1
                                time.sleep(0.5)
                            else:
                                rospy.loginfo("Detected obstacles, continue to left wall following")
                                self.oa_turn= False
                        if (self.lf == 0):
                            self.oa = False
                            self.simple_vel.v = 0
                            self.simple_vel.w = 1.5
                            self.set_velocities_pub.publish(self.simple_vel)
                            time.sleep(0.5)
                            self.simple_vel.v = 0.05 # 0.08
                            self.simple_vel.w = 0
                            self.set_velocities_pub.publish(self.simple_vel)
                            time.sleep(0.5)
                            rospy.loginfo("Exiting Obstacle Avoidance")
                    # update error
                    self.prev_yaw_err, self.prev_pos_err = self.yaw_err, self.pos_err
            self.set_velocities_pub.publish(self.simple_vel)
            loop_rate.sleep()

if __name__ == '__main__':
    pi = math.pi
    tc = ThymioController()
    try:
        # modes: run_with_obstacle_avoidance | calibrate_rot | stop_read
        tc.run_with_obstacle_avoidance()
    except rospy.ROSInterruptException:
        pass
