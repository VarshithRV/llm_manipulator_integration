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

def plan_to_object():
    rospy.init_node("moveit_planner", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2.0)

    try:
        trans_obj = tf_buffer.lookup_transform('camera_frame', 'object_frame', rospy.Time(0.0))
        trans_base = tf_buffer.lookup_transform('camera_frame', 'base_link', rospy.Time(0.0))
    except tf2_ros.TransformException as e:
        rospy.logerr("Cannot transform: %s" % e)
        return
    
    obj_in_base = tf_buffer.transform(trans_obj, 'base_link')

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = obj_in_base.transform.translation.x
    target_pose.position.y = obj_in_base.transform.translation.y
    target_pose.position.z = obj_in_base.transform.translation.z
    target_pose.orientation = obj_in_base.transform.rotation

    move_group.set_pose_target(target_pose)
    # Planning trajectory
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    current_pose = move_group.get_current_pose().pose
    success = all_close(target_pose, current_pose, 0.01)
    if success:
        rospy.loginfo("Successful planning")
    else:
        rospy.logwarn("Failed to execute")

if __name__ == '__main__':
    try:
        plan_to_object()
    except rospy.ROSInterruptException:
        pass
