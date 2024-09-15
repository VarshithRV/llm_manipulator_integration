# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped
from motion_planning_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from motion_planning_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib

class MpClass:
    def __init__(self):
        self.clean_client = actionlib.SimpleActionClient("clean", PickPlaceAction)
        self.clean_client.wait_for_server()


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    dummy = PointStamped()
    clean_goal = PickPlaceGoal()
    clean_goal.source = dummy
    clean_goal.destination = dummy
    rospy.loginfo("Sending clean goal")
    mp.clean_client.send_goal(clean_goal)
    mp.clean_client.wait_for_result()
    clean_result = mp.clean_client.get_result()
    print("clean result : ", clean_result.result)
    print("Done")