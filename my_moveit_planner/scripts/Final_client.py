#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from my_moveit_planner.msg import FinalAction, FinalGoal

def send_goal():
    client = actionlib.SimpleActionClient('task_server', FinalAction)
    client.wait_for_server()

    goal = FinalGoal()

    # Define start_pose
    goal.start_pose = PointStamped()
    goal.start_pose.header.frame_id = "base_link"
    goal.start_pose.header.stamp = rospy.Time.now()
    goal.start_pose.point.x = -0.2397
    goal.start_pose.point.y = -0.6992
    goal.start_pose.point.z = 0.150

    # Define end_pose
    goal.end_pose = PointStamped()
    goal.end_pose.header.frame_id = "base_link"
    goal.end_pose.header.stamp = rospy.Time.now()
    goal.end_pose.point.x = -0.2397
    goal.end_pose.point.y = -0.6992
    goal.end_pose.point.z = 0.200

    rospy.loginfo("Sending goal to action server")
    client.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)

    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo(f"Result: {result.success}")

def done_cb(state, result):
    rospy.loginfo(f"Action completed with state: {state}, result: {result.success}")

def active_cb():
    rospy.loginfo("Action is active")

def feedback_cb(feedback):
    rospy.loginfo(f"Current pose: {feedback.current_pose}")

if __name__ == '__main__':
    try:
        rospy.init_node('point_sender')
        send_goal()
    except rospy.ROSInterruptException:
        pass
