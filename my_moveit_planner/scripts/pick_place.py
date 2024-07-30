import rospy
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np 
import sys

def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for i in range(len(goal)):
            if abs(actual[i] - goal[i]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

def move_between_points(point1, point2, move_group):
    move_group.set_pose_target(point1)
    move_group.go(wait=True)
    move_group.set_pose_target(point2)
    move_group.go(wait=True)

def plan_to_object():
    rospy.init_node("moveit_planner", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    point1 = geometry_msgs.msg.Pose()
    point1.position.x = 0.5  # Adjust x, y, z coordinates as needed
    point1.position.y = 0.5
    point1.position.z = 0.5
    point1.orientation.w = 1.0  # Quaternion components

    point2 = geometry_msgs.msg.Pose()
    point2.position.x = -0.5  # Adjust x, y, z coordinates as needed
    point2.position.y = -0.5
    point2.position.z = 0.5
    point2.orientation.w = 1.0  # Quaternion components

    # Loop to move between points
    while not rospy.is_shutdown():
        move_between_points(point1, point2, move_group)

if __name__ == '__main__':
    try:
        plan_to_object()
    except rospy.ROSInterruptException:
        pass
