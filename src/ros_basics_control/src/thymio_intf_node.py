#!/usr/bin/env python3

from thymiodirect import Connection
from thymiodirect import Thymio

import rospy
from ros_basics_msgs.msg import SimpleVelocities
from ros_basics_msgs.msg import ProximitySensors

import numpy as np

from geometry_msgs.msg import Twist


class ThymioInterface:
    def __init__(self):
        self._port = Connection.serial_default_port()
        self._th = Thymio(
            serial_port=self._port,
            on_connect=lambda node_id: print(f"{node_id} is connected"))
        self._th.connect()
        self._id = self._th.first_node()

        rospy.init_node('thymio_intf_node', anonymous=True)
        self._psensor_pub = rospy.Publisher(
            'proximity_sensors', ProximitySensors, queue_size=10)
        rospy.Subscriber("set_velocities", SimpleVelocities,
                         self._vel_callback)
        rospy.on_shutdown(self._shutdown)

        self._lb_new = -500.
        self._ub_new = 500.
        self._lb_old = -0.2
        self._ub_old = 0.2
        self._wheel_rad = 0.022
        self._wheel_dist_2 = 0.0475
        self._regu = 45  # TODO: check this

    def spin(self):
        loop_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish_psensors(self._th[self._id]["prox.horizontal"])
            loop_rate.sleep()

    def _publish_psensors(self, values):
        ps = ProximitySensors()
        ps.header.stamp = rospy.Time.now()
        ps.values = values
        self._psensor_pub.publish(ps)

    def _vel_callback(self, data):
        l_speed = (data.v + self._wheel_dist_2 * -data.w) / \
            self._wheel_rad
        r_speed = (data.v - self._wheel_dist_2 * -data.w) / \
            self._wheel_rad
        l_speed /= self._regu
        r_speed /= self._regu
        l_speed = self._rescale(l_speed)
        r_speed = self._rescale(r_speed)
        self._move(l_speed, r_speed)

    def _move(self, l_speed, r_speed):
        #print("(l, r): ({}, {})".format(l_speed, r_speed))
        l_speed = l_speed if l_speed >= 0 else 2 ** 16 + l_speed
        r_speed = r_speed if r_speed >= 0 else 2 ** 16 + r_speed
        self._th[self._id]["motor.left.target"] = l_speed
        self._th[self._id]["motor.right.target"] = r_speed

    def _rescale(self, val):
        if val > self._ub_old:
            val = self._ub_old
        elif val < self._lb_old:
            val = self._lb_old
        nrange = self._ub_new - self._lb_new
        orange = self._ub_old - self._lb_old
        return int((((val - self._lb_old) * nrange) / orange) + self._lb_new)

    def _shutdown(self):
        self._move(0, 0)


if __name__ == '__main__':
    ti = ThymioInterface()
    ti.spin()
