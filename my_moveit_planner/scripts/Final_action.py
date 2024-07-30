#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, PointStamped
from my_moveit_planner.msg import FinalAction, FinalResult, FinalFeedback
from trajectory_msgs.msg import JointTrajectory
import moveit_commander
from moveit_commander.exception import MoveItCommanderException
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool

Flag = False
Gripper_flag = False

class TaskServer:
    def __init__(self, name):
        self.action_name_ = name
        moveit_commander.roscpp_initialize([])
        rospy.init_node('task_server')
        self.arm_move_group_ = moveit_commander.MoveGroupCommander("manipulator")
        self.arm_move_group_.set_planning_time(10.0)  # Increase allowed planning time
        self.tool_position_publisher_ = rospy.Publisher('/move_group/tool_position', PoseStamped, queue_size=10)
        self.as_ = actionlib.SimpleActionServer(self.action_name_, FinalAction, execute_cb=self.execute_cb, auto_start=False)
        self.as_.start()

        # Initialize ROS service wrapper

    def execute_cb(self, goal):
        global Flag, Gripper_flag
        success = True

        try:
            # Get the start position from the start_pose
            start_pose = goal.start_pose.point

            # Log received start position
            rospy.loginfo(f"Received start position: x={start_pose.x}, y={start_pose.y}, z={start_pose.z}")

            # Move 70 mm above the start position
            above_start_pose = PoseStamped()
            above_start_pose.header.frame_id = "base_link"
            above_start_pose.pose.position.x = start_pose.x
            above_start_pose.pose.position.y = start_pose.y
            above_start_pose.pose.position.z = start_pose.z + 0.07
            above_start_pose.pose.orientation.w = 1.0

            self.arm_move_group_.set_pose_target(above_start_pose)

            # Plan to the new pose
            plan = self.arm_move_group_.plan()
            if not plan or len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Planning to above start pose failed")
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            # Execute the plan to the above start position
            rospy.loginfo("Executing plan to above start position")
            success = self.arm_move_group_.execute(plan, wait=True)
            if not success:
                rospy.logerr("Execution to above start pose failed")
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            self.arm_move_group_.stop()
            self.arm_move_group_.clear_pose_targets()

            # Check for preemption
            if self.as_.is_preempt_requested():
                rospy.loginfo('%s is Preempted' % self.action_name_)
                self.as_.set_preempted()
                return

            # Move down to the start position
            pose_target = PoseStamped()
            pose_target.header.frame_id = "base_link"
            pose_target.pose.position.x = start_pose.x
            pose_target.pose.position.y = start_pose.y
            pose_target.pose.position.z = start_pose.z
            pose_target.pose.orientation.w = 1.0

            self.arm_move_group_.set_pose_target(pose_target)

            # Plan to the new pose
            plan = self.arm_move_group_.plan()
            if not plan or len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Planning to start pose failed")
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            # Execute the plan to the start position
            rospy.loginfo("Executing plan to start position")
            success = self.arm_move_group_.execute(plan, wait=True)
            if not success:
                rospy.logerr("Execution to start pose failed")
                self.result_.success = False
                self.as_.set_aborted(self.result_)
                return

            self.arm_move_group_.stop()
            self.arm_move_group_.clear_pose_targets()

            # Publish the current tool position at the start
            tool_position = self.arm_move_group_.get_current_pose()
            self.tool_position_publisher_.publish(tool_position)

            # Log current tool position
            rospy.loginfo(f"Published start tool position: x={tool_position.pose.position.x}, y={tool_position.pose.position.y}, z={tool_position.pose.position.z}, "
                          f"rx={tool_position.pose.orientation.x}, ry={tool_position.pose.orientation.y}, rz={tool_position.pose.orientation.z}, rw={tool_position.pose.orientation.w}")

            # Check for preemption
            if self.as_.is_preempt_requested():
                rospy.loginfo('%s is Preempted' % self.action_name_)
                self.as_.set_preempted()
                return

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
            rospy.loginfo('Successfully reached start position')

            # Set the flag to True and call the Arduino service to control the gripper
            Flag = True
            rospy.loginfo("Calling ROS service to control the gripper")
            try:
                response = self.service_wrapper.set_gripper_state(True) # Set the gripper state to True
                if response.success:
                    rospy.loginfo("Successfully controlled the gripper")

                    # Publish False to /gripper topic after executing the gripper control
                    self.service_wrapper.publish_gripper_state(False)
                    rospy.loginfo("Published False to /gripper topic")
                    Gripper_flag = True  # Set Gripper_flag to True after controlling the gripper
                else:
                    rospy.logerr("Failed to control the gripper")
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

        if Gripper_flag:
            try:
                # Get the end position from the end_pose
                end_pose = goal.end_pose.point

                # Log received end position
                rospy.loginfo(f"Received end position: x={end_pose.x}, y={end_pose.y}, z={end_pose.z}")

                # Create a pose target for the end position
                pose_target = PoseStamped()
                pose_target.header.frame_id = "base_link"
                pose_target.pose.position.x = end_pose.x
                pose_target.pose.position.y = end_pose.y
                pose_target.pose.position.z = end_pose.z
                pose_target.pose.orientation.w = 1.0

                self.arm_move_group_.set_pose_target(pose_target)

                # Plan to the new pose
                plan = self.arm_move_group_.plan()
                if not plan or len(plan.joint_trajectory.points) == 0:
                    rospy.logerr("Planning to end pose failed")
                    self.result_.success = False
                    self.as_.set_aborted(self.result_)
                    return

                # Execute the plan to the end position
                rospy.loginfo("Executing plan to end position")
                success = self.arm_move_group_.execute(plan, wait=True)
                if not success:
                    rospy.logerr("Execution to end pose failed")
                    self.result_.success = False
                    self.as_.set_aborted(self.result_)
                    return

                self.arm_move_group_.stop()
                self.arm_move_group_.clear_pose_targets()

                # Move 70 mm above the end position
                above_end_pose = PoseStamped()
                above_end_pose.header.frame_id = "base_link"
                above_end_pose.pose.position.x = end_pose.x
                above_end_pose.pose.position.y = end_pose.y
                above_end_pose.pose.position.z = end_pose.z + 0.07
                above_end_pose.pose.orientation.w = 1.0

                self.arm_move_group_.set_pose_target(above_end_pose)

                # Plan to the new pose
                plan = self.arm_move_group_.plan()
                if not plan or len(plan.joint_trajectory.points) == 0:
                    rospy.logerr("Planning to above end pose failed")
                    self.result_.success = False
                    self.as_.set_aborted(self.result_)
                    return

                # Execute the plan to the above end position
                rospy.loginfo("Executing plan to above end position")
                success = self.arm_move_group_.execute(plan, wait=True)
                if not success:
                    rospy.logerr("Execution to above end pose failed")
                    self.result_.success = False
                    self.as_.set_aborted(self.result_)
                    return

                self.arm_move_group_.stop()
                self.arm_move_group_.clear_pose_targets()

                # Publish the current tool position at the above end position
                tool_position = self.arm_move_group_.get_current_pose()
                self.tool_position_publisher_.publish(tool_position)

                # Log current tool position
                rospy.loginfo(f"Published above end tool position: x={tool_position.pose.position.x}, y={tool_position.pose.position.y}, z={tool_position.pose.position.z}, "
                              f"rx={tool_position.pose.orientation.x}, ry={tool_position.pose.orientation.y}, rz={tool_position.pose.orientation.z}, rw={tool_position.pose.orientation.w}")

                # Check for preemption
                if self.as_.is_preempt_requested():
                    rospy.loginfo('%s is Preempted' % self.action_name_)
                    self.as_.set_preempted()
                    return

                if success:
                    self.result_.success = True
                    self.as_.set_succeeded(self.result_)
                    rospy.loginfo('%s: Succeeded' % self.action_name_)

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

if __name__ == "__main__":
    try:
        TaskServer('task_server')
        rospy.loginfo("Task Server Started")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted by user")
