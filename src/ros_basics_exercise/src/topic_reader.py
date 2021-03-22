#!/usr/bin/env python
"""
read data from topic /robot_pose and /proximity_sensors in the recorded rosbags
"""
import rospy
import math
import time
import numpy
import os
from ros_basics_msgs.msg import SimplePoseStamped
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.msg import ProximitySensors

class TopicReader:
    def __init__(self, mode, bag_name):
        self.mode = mode
        self.folder = os.getcwd() + '/results_from_bag/'
        self.bag_name = bag_name
        self.mode_folder = self.folder  + '/' + self.mode + '/'
        self.bag_folder = self.folder + '/' + self.mode + '/' + self.bag_name  + '/'
        if not os.path.exists(self.folder):
            os.mkdir(self.folder)
        if not os.path.exists(self.mode_folder):
            os.mkdir(self.mode_folder)
        if not os.path.exists(self.bag_folder):
            os.mkdir(self.bag_folder)
        # timestamp
        self.timestamp = 0.0
        # pose data
        #self.pose = SimplePoseStamped()
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.timestamp = 0
        # sensor data
        #self.proximity = ProximitySensors()
        self.prox_h0 = 0.  # float
        self.prox_h1 = 0.  # float
        self.prox_h2 = 0.  # float
        self.prox_h3 = 0.  # float
        self.prox_h4 = 0.  # float

        # Init node
        rospy.init_node('topic_reader', anonymous=True)
        rospy.loginfo('Node "topic_reader" created')

        # Create subscribers
        rospy.loginfo('Create robot_pose_sub')  # average rate: 10.00
        self.robot_pose_sub = rospy.Subscriber('robot_pose', SimplePoseStamped, self.robot_pose_cb)
        rospy.loginfo('Create proximity_sub')  # average rate: 2.500
        self.proximity_sub = rospy.Subscriber('proximity_sensors', ProximitySensors, self.proximity_cb)

    def robot_pose_cb(self, simple_pose_stamped):
        """subscribe to the robot_pose topic to get the robot's current pose"""
        #self.pose = simple_pose_stamped
        self.x = simple_pose_stamped.pose.xyz.x
        self.y = simple_pose_stamped.pose.xyz.y
        self.yaw = simple_pose_stamped.pose.rpy.yaw
        self.timestamp = simple_pose_stamped.header.stamp.secs \
                                         + simple_pose_stamped.header.stamp.nsecs * 1e-9

    def proximity_cb(self, proximity_sensors):
        """subscribe to the proximity_sensors topic to get the values of proximity sensors"""
        #self.proximity = proximity_sensors
        self.prox_h0 = proximity_sensors.values[0]
        self.prox_h1 = proximity_sensors.values[1]
        self.prox_h2 = proximity_sensors.values[2]
        self.prox_h3 = proximity_sensors.values[3]
        self.prox_h4 = proximity_sensors.values[4]

    def read_pose_proximity(self):
        """stop Thymio to read proximity sensor value for real-world deployment"""
        loop_rate = rospy.Rate(10)  # Hz
        x_list, y_list, yaw_list, time_list = [], [], [], []
        prox_h0_list, prox_h1_list, prox_h2_list, prox_h3_list , prox_h4_list= [], [], [], [], []
        while not rospy.is_shutdown():
            #rospy.loginfo("x: {}, y: {}, yaw: {}".format(self.x, self.y, self.yaw))
            #rospy.loginfo("sensor0: {}, sensor1: {}, sensor2: {}, sensor3: {}, sensor4: {}, ".format(self.prox_h0, self.prox_h1, self.prox_h2, self.prox_h3, self.prox_h4))
            x_list.append(self.x)
            y_list.append(self.y)
            yaw_list.append(self.yaw)
            time_list.append(self.timestamp)
            prox_h0_list.append(self.prox_h0)
            prox_h1_list.append(self.prox_h1)
            prox_h2_list.append(self.prox_h2)
            prox_h3_list.append(self.prox_h3)
            prox_h4_list.append(self.prox_h4)
            loop_rate.sleep()
        # converting list to array
        numpy.savetxt(self.bag_folder+'x_data.csv', numpy.array(x_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'y_data.csv', numpy.array(y_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'yaw_data.csv', numpy.array(yaw_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'timestamp_data.csv', numpy.array(time_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'h0_data.csv', numpy.array(prox_h0_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'h1_data.csv', numpy.array(prox_h1_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'h2_data.csv', numpy.array(prox_h2_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'h3_data.csv', numpy.array(prox_h3_list), delimiter=',')
        numpy.savetxt(self.bag_folder+'h4_data.csv', numpy.array(prox_h4_list), delimiter=',')

if __name__ == '__main__':
    mode = 'real' # simu | real
    """
    simu: thymio_simulation_navigation_with_obstacle_avoidance
    real: thymio_real_with_obstacle_avoidance | thymio_real_without_obstacle
    """
    bag_name = 'thymio_real_without_obstacle'
    tv = TopicReader(mode, bag_name)
    try:
        # run directly tc.run_with_obstacle_avoidance()
        # run with different modes tc.run()
        tv.read_pose_proximity()
    except rospy.ROSInterruptException:
        pass
