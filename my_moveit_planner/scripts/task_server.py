#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander
import sys

class ToolPositionPublisher(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('tool_position_publisher')
        self.arm_move_group_ = moveit_commander.MoveGroupCommander("manipulator")
        self.tool_position_publisher_ = rospy.Publisher('/move_group/tool_position', PoseStamped, queue_size=10)
        self.publish_rate_ = rospy.Rate(10)  # 10 Hz

    def publish_tool_position(self):
        while not rospy.is_shutdown():
            try:
                # Get the current tool position
                tool_position = self.arm_move_group_.get_current_pose()
                # Publish the tool position
                self.tool_position_publisher_.publish(tool_position)
                rospy.loginfo(f"Published tool position: x={tool_position.pose.position.x}, y={tool_position.pose.position.y}, z={tool_position.pose.position.z}, "
                              f"rx={tool_position.pose.orientation.x}, ry={tool_position.pose.orientation.y}, rz={tool_position.pose.orientation.z}, rw={tool_position.pose.orientation.w}")
            except moveit_commander.MoveItCommanderException as e:
                rospy.logerr("MoveItCommanderException: %s" % str(e))
            except Exception as e:
                rospy.logerr("Unexpected exception: %s" % str(e))
            # Sleep for the defined rate
            self.publish_rate_.sleep()

if __name__ == '__main__':
    try:
        publisher = ToolPositionPublisher()
        publisher.publish_tool_position()
    except rospy.ROSInterruptException:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
