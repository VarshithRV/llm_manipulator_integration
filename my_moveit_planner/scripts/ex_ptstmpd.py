#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from my_moveit_planner.msg import PickAction, PickGoal

def send_goal():
    client = actionlib.SimpleActionClient('task_server', PickAction)
    client.wait_for_server()

    goal = PickGoal()
    goal.end_pose = PointStamped()
    goal.end_pose.header.frame_id = "base_link"
    goal.end_pose.header.stamp = rospy.Time.now()
    goal.end_pose.point.x = -0.1998
    goal.end_pose.point.y = -0.6405
    goal.end_pose.point.z = 0.1538
    #goal.end_pose.point.x = -0.3791
    #goal.end_pose.point.y = -0.6252
    #goal.end_pose.point.z = 0.2044

    rospy.loginfo("Sending goal to action server")
    client.send_goal(goal)

    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo(f"Result: {result.success}")

if __name__ == '__main__':
    try:
        rospy.init_node('point_sender')
        send_goal()
    except rospy.ROSInterruptException:
        pass
