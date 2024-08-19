# write a an action client for the pick and place action server

import rospy
from geometry_msgs.msg import PointStamped
from motion_planning_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from motion_planning_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse

class MpClass:
    def __init__(self):
        self.pick_place_client = actionlib.SimpleActionClient("pick_place", PickPlaceAction)
        self.move_preaction_client = actionlib.SimpleActionClient("move_preaction", MovePreactionAction)
        self.pick_place_client.wait_for_server()
        self.move_preaction_client.wait_for_server()


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()

    rospy.loginfo("Sending move preaction goal")
    move_preaction_goal = MovePreactionActionGoal()
    mp.move_preaction_client.send_goal(move_preaction_goal)
    mp.move_preaction_client.wait_for_result()
    move_preaction_result = mp.move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)
    
    print("Done")