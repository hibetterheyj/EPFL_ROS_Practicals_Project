#!/usr/bin/env python3

import rospy
# from ros_basics_msgs.msg import SimplePoseStamped
# from ros_basics_msgs.srv import CheckWaypointReached
from ros_basics_msgs.srv import SetWaypoints
from geometry_msgs.msg import Pose2D


## Trajectory to follow
x_w = [0.1002142315,
0.20644131689,
0.17236847817999998,
0.09319923529499999,
0.0,
-0.08417995445999998,
0.001002142315,
-0.1603427704,
-0.1603427704]

y_w = [0.0030064269449999997,
0.0030064269449999997,
0.0801713852,
0.11825279317,
0.11725065085499999,
-0.11825279317,
-0.11624850853999999,
-0.06814567742,
0.051109258064999996,
0.121259220115]

rospy.init_node('create_trajectory', anonymous=True)
# loop_rate = rospy.Rate(10)

print("Adding new waypoints")

pose_l = []

for i, x in enumerate(x_w):
    p = Pose2D()
    p.x = x_w[i]
    p.y = y_w[i]

    pose_l.append( p )

rospy.wait_for_service('set_waypoints')

try:
    set_wpt = rospy.ServiceProxy('set_waypoints', SetWaypoints)

    resp = set_wpt( pose_l )
    print(resp)
    print(type(resp))

except rospy.ServiceException as e:
    print("Service call failed: %s"%e)

print("Yeah! Waypoints added. You can kill me now.")
# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
