#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped
from my_moveit_planner.msg import PickAction, PickResult, PickFeedback
from trajectory_msgs.msg import JointTrajectory
import sys
import moveit_commander
from moveit_commander.exception import MoveItCommanderException
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

Flag = False

class TaskServer(object):
    result_ = PickResult()
    feedback_ = PickFeedback()

    def __init__(self, name):
        self.action_name_ = name
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('task_server')
        self.arm_move_group_ = moveit_commander.MoveGroupCommander("manipulator")
        self.joint_angles_publisher_ = rospy.Publisher('/move_group/desired_joint_angles', JointTrajectory, queue_size=10)
        self.tool_position_publisher_ = rospy.Publisher('/move_group/tool_position', PoseStamped, queue_size=10)
        self.as_ = actionlib.SimpleActionServer(self.action_name_, PickAction, execute_cb=self.execute_cb, auto_start=False)
        self.as_.start()

        # Initialize ROS service wrapper
        self.service_wrapper = RosServiceWrapper()

    def execute_cb(self, goal):
        global Flag
        success = True

        try:
            # Get the goal position from the end_pose
            target_pose = goal.end_pose.point

            # Log received goal
            rospy.loginfo(f"Received goal position: x={target_pose.x}, y={target_pose.y}, z={target_pose.z}")

            # Create a pose target
            pose_target = self.arm_move_group_.get_current_pose().pose
            pose_target.position.x = target_pose.x
            pose_target.position.y = target_pose.y
            pose_target.position.z = target_pose.z
            pose_target.orientation.x = 0.0
            pose_target.orientation.y = 1.0  
            pose_target.orientation.z = 0.0
            pose_target.orientation.w = 0.0 

            self.arm_move_group_.set_pose_target(pose_target)

            # Plan to the new pose
            plan = self.arm_move_group_.plan()
            if isinstance(plan, tuple):
                plan = plan[1]

            if not plan or len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Planning failed")
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            # Publish the end position joint angles
            end_joint_angles = plan.joint_trajectory.points[-1]
            joint_trajectory = JointTrajectory()
            joint_trajectory.joint_names = plan.joint_trajectory.joint_names
            joint_trajectory.points.append(end_joint_angles)
            self.joint_angles_publisher_.publish(joint_trajectory)
            rospy.loginfo(f"Published end joint angles: {end_joint_angles.positions}")

            # Execute the plan
            rospy.loginfo("Executing plan")
            success = self.arm_move_group_.execute(plan, wait=True)
            if not success:
                rospy.logerr("Execution failed")
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            self.arm_move_group_.stop()
            self.arm_move_group_.clear_pose_targets()

            # Provide feedback during the movement
            current_pose = self.arm_move_group_.get_current_pose().pose
            self.feedback_.current_pose.point.x = current_pose.position.x
            self.feedback_.current_pose.point.y = current_pose.position.y
            self.feedback_.current_pose.point.z = current_pose.position.z
            self.as_.publish_feedback(self.feedback_)

            # Publish the current tool position
            tool_position = self.arm_move_group_.get_current_pose()
            self.tool_position_publisher_.publish(tool_position)
            rospy.loginfo(f"Published tool position: x={tool_position.pose.position.x}, y={tool_position.pose.position.y}, z={tool_position.pose.position.z}, "
                          f"rx={tool_position.pose.orientation.x}, ry={tool_position.pose.orientation.y}, rz={tool_position.pose.orientation.z}, rw={tool_position.pose.orientation.w}")

            if self.as_.is_preempt_requested():
                rospy.loginfo('%s is Preempted' % self.action_name_)
                self.as_.set_preempted()
                success = False

        except MoveItCommanderException as e:
            rospy.logerr("MoveItCommanderException: %s" % str(e))
            self.result_.success = False
            self.as_.set_aborted(self.result_)
            return
        except Exception as e:
            rospy.logerr("Unexpected exception: %s" % str(e))
            self.result_.success = False
            self.as_.set_aborted(self.result_)
            return

        if success:
            self.result_.success = True
            rospy.loginfo('%s Succeeded' % self.action_name_)
            self.as_.set_succeeded(self.result_)

            # Set the flag to True and call the Arduino service to control the pin
            Flag = True
            rospy.loginfo("Calling ROS service to control the gripper")
            try:
                response = self.service_wrapper.set_gripper_state(True)  # Set the gripper state to True
                if response.success:
                    rospy.loginfo("Successfully controlled the gripper")

                    # Publish False to /gripper topic after executing the trajectory planning
                    self.service_wrapper.set_gripper_state(False)
                    rospy.loginfo("Published False to /gripper topic")
                else:
                    rospy.logerr("Failed to control the gripper")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

class RosServiceWrapper:
    def __init__(self):
        rospy.wait_for_service('/gripper')
        self.set_gripper_state = rospy.ServiceProxy('/gripper', SetBool)
        self.setpoint_pub = rospy.Publisher('/gripper', Bool, queue_size=1)

    def publish_gripper_state(self, state):
        self.setpoint_pub.publish(Bool(data=state))

if __name__ == '__main__':
    server = TaskServer('task_server')
    rospy.spin()
