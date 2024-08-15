#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from my_moveit_planner.msg import FinalAction, FinalGoal
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests

class CentralClient:
    def __init__(self):
        self.action_client = actionlib.SimpleActionClient('task_server', FinalAction)

    
    def send_goal(self, start_pose, end_pose):
        self.action_client.wait_for_server()
        goal = FinalGoal()
        goal.start_pose = start_pose
        goal.end_pose = end_pose

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()

    # execute all the actions in the action list one by one here.
    def execute_actions(self, action_list):
        print("Executing actions ...")
        for action in action_list:
            print("Executing action : ", action["action_type"])
            print("Source object position : ")
            print(action["source_object_position"])
            print("Target object position : ") 
            print(action["target_object_position"])

            start_pose = PointStamped()
            start_pose.header.frame_id = "base_link"
            start_pose.header.stamp = rospy.Time.now()
            start_pose.point.x = action["source_object_position"].point.x
            start_pose.point.y = action["source_object_position"].point.y
            start_pose.point.z = action["source_object_position"].point.z + 0.120

            end_pose = PointStamped()
            end_pose.header.frame_id = "base_link"
            end_pose.header.stamp = rospy.Time.now()
            end_pose.point.x = action["target_object_position"].point.x 
            end_pose.point.y = action["target_object_position"].point.y
            end_pose.point.z = action["target_object_position"].point.z  + 0.20

            result = self.send_goal(start_pose, end_pose)
            if not result.success:
                print("Action failed")
                break

if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    # # Call the service, response contains object_positions, bounding_boxes, and image
    # response = central_client.get_object_locations()
    # if not response:
    #     rospy.logerr("Failed to get object locations")
    #     exit(1)

    # rospy.sleep(0.5)
    # instruction = "pick up all the oranges and place it in a bowl"
    # print("Requesting for plan ...")
    # print("Prompt : ", instruction)

    # plan = central_client.test_demo_api(instruction, response)
    # for action in plan["plan_actions"]:
    #     action["source_object_id"] = int(action["source_object_id"])
    #     action["target_object_id"] = int(action["target_object_id"])

    # for action in plan["plan_actions"]:
    #     action_parsed = {
    #         "action_type": action["action_type"],
    #         "source_object_position": response.result.object_position[action["source_object_id"]].position,
    #         "target_object_position": response.result.object_position[action["target_object_id"]].position
    #     }
    #     action_list.append(action_parsed)
    
    action_list = []
    
    source_1_position = PointStamped()
    source_2_position = PointStamped()
    source_3_position = PointStamped()
    destination_position = PointStamped()


    # hardcoded source and destination positions
    source_1_position.point.x = -0.11079056151090363
    source_1_position.point.y = -0.4831845311843534
    source_1_position.point.z = 0.04554612949580239
    source_2_position.point.x =  -0.25055244338918636
    source_2_position.point.y = -0.6880319654187088
    source_2_position.point.z = 0.030098766974563707
    source_3_position.point.x =  -0.1373833868751858
    source_3_position.point.y = -0.7356613322736578
    source_3_position.point.z = 0.05292135332072356
    destination_position.point.x = -0.2912284669589617
    destination_position.point.y = -0.3669910503400372
    destination_position.point.z = 0.014983440602635567

    plan1 = {"action_type":"pick_and_place", 
             "source_object_position":source_1_position,
             "target_object_position":destination_position
             }
    plan2 = {"action_type":"pick_and_place", 
             "source_object_position":source_2_position,
             "target_object_position":destination_position
             }
    plan3 = {"action_type":"pick_and_place", 
             "source_object_position":source_3_position,
             "target_object_position":destination_position
             }
    
    action_list = [plan1,plan2,plan3]
    

    central_client.execute_actions(action_list)
