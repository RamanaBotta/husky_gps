#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, Quaternion
import actionlib
import importlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from tf.transformations import quaternion_from_euler

rospy.init_node('odom_gofetch')
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()
a = 5
list_traverse = \
[
    [a,a],
    [a,-a],
    [-a,a],
    [-a,a]
]

while True:
    for val in list_traverse:
        print(val)

        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = 'odom'
        move_base_goal.target_pose.header.stamp = rospy.Time(0)
        move_base_goal.target_pose.pose.position.x = val[0]
        move_base_goal.target_pose.pose.position.y = val[1]
        move_base_goal.target_pose.pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, 0.0))


        client.send_goal(move_base_goal)

        wait = client.wait_for_result()
        print(wait)
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
                print(client.get_result())

