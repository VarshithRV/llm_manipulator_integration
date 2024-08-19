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
        # self.get_object_locations_service = rospy.ServiceProxy(
        #     "get_object_locations",
        #     GetObjectLocations
        # )
    
    # def get_object_locations(self):
    #     try:
    #         response = self.get_object_locations_service()
    #         return response
    #     except rospy.ServiceException as e:
    #         print(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node("motion_planning_client")
    mp = MpClass()
    source = PointStamped()
    source.point.x = -0.11079056151090363
    source.point.y = -0.4831845311843534
    source.point.z = 0.04554612949580239 #+ 0.20
    destination = PointStamped()
    destination.point.x = -0.2912284669589617
    destination.point.y = -0.3669910503400372
    destination.point.z = 0.014983440602635567 #+ 0.20

    # detect objects
    # source_position = PointStamped()
    # destination_position = PointStamped()
    # object_name = "carrot"

    # response = mp.get_object_locations()
    # for object in response.result.object_position:
    #     print(object.object_name)
    #     print(object.object_position)

    rospy.loginfo("Sending pick and place goal")
    pick_place_goal = PickPlaceGoal()
    pick_place_goal.source = source
    pick_place_goal.destination = destination

    rospy.loginfo("Sending move preaction goal")
    move_preaction_goal = MovePreactionActionGoal()
    mp.move_preaction_client.send_goal(move_preaction_goal)
    mp.move_preaction_client.wait_for_result()
    move_preaction_result = mp.move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)

    mp.pick_place_client.send_goal(pick_place_goal)
    mp.pick_place_client.wait_for_result()
    pick_place_result = mp.pick_place_client.get_result()
    print("Pick and place result : ", pick_place_result.result)

    rospy.loginfo("Sending move preaction goal")
    move_preaction_goal = MovePreactionActionGoal()
    mp.move_preaction_client.send_goal(move_preaction_goal)
    mp.move_preaction_client.wait_for_result()
    move_preaction_result = mp.move_preaction_client.get_result()
    print("Move preaction result : ", move_preaction_result.result)
    print("Done")