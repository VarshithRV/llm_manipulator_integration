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
        self.get_object_locations_service = rospy.ServiceProxy("get_object_locations", GetObjectLocations)
        self.action_client = actionlib.SimpleActionClient('task_server', FinalAction)

    def get_object_locations(self):
        try:
            response = self.get_object_locations_service()
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def convert_image_to_text(self, image: Image) -> str:
        with io.BytesIO() as output:
            image.save(output, format="PNG")
            data = output.getvalue()
        return base64.b64encode(data).decode("utf-8")
    
    def test_demo_api(
        self,
        instruction: str,
        response: GetObjectLocationsResponse,
        url: str = "https://robot-api-2.glitch.me/handle_request",
        ):
        

        image_ = cv_bridge.CvBridge().imgmsg_to_cv2(response.result.image, desired_encoding="bgr8")
        image_ = Image.fromarray(cv2.cvtColor(image_, cv2.COLOR_BGR2RGB))

        # create a timestamp using ros
        timestamp = rospy.Time.now()
        image_.save(str(timestamp)+".png")

        objects = []
        for object_position in response.result.object_position:
            objects.append(
                {
                    "object_id": object_position.id,
                    "x_min": object_position.x_min,
                    "y_min": object_position.y_min,
                    "x_max": object_position.x_max,
                    "y_max": object_position.y_max,
                }
            )  
        
        # print("Calling API with objects : ",response.result.object_position)
        # for object in objects, object_position in response.result.object_position:
        #     print("Object id :",object["object_id"])
        #     print("Class : ",object_position.Class)

        image = {
            "base64_string": self.convert_image_to_text(image_),
            "objects": objects
        }

        data = {
            "instruction": instruction,
            "images": [image],
        }
        
        print(data["instruction"], image["objects"])
        headers = {"Content-Type": "application/json"}
        content = dict(data=data, job_type="planning")
        response = requests.post(url, data=json.dumps(content), headers=headers)
        return response.json()
    
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
            start_pose.point.z = action["source_object_position"].point.z + 0.12

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
    
    # Call the service, response contains object_positions, bounding_boxes, and image
    response = central_client.get_object_locations()
    if not response:
        rospy.logerr("Failed to get object locations")
        exit(1)

    for object_thing in response.result.object_position:
        print("Object ID : ", object_thing.id)
        print("Object Class : ", object_thing.Class)


    instruction = input("Enter the prompt : ")
    # instruction = "pick up all the oranges and place it in a bowl"
    print("Requesting for plan ...")
    print("Prompt : ", instruction)

    plan = central_client.test_demo_api(instruction, response)
    for action in plan["plan_actions"]:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])
    
    print(plan["plan_actions"])

    input("Press Enter to continue ...")

    action_list = []
    # substitute the object class specific position offsets here
    for action in plan["plan_actions"]:
        # get the class of the object for the source object
        source_object_class = response.result.object_position[action["source_object_id"]].Class
        if source_object_class == "orange":
            response.result.object_position[action["source_object_id"]].position.point.z += 0
        if source_object_class == "carrot":
            response.result.object_position[action["source_object_id"]].position.point.z -= 0.008
        if source_object_class == "apple":
            response.result.object_position[action["source_object_id"]].position.point.z -= 0.017
        
        action_parsed = {
            "action_type": action["action_type"],
            "source_object_position": response.result.object_position[action["source_object_id"]].position,
            "target_object_position": response.result.object_position[action["target_object_id"]].position
        }
        action_list.append(action_parsed)

    # prints the raw action list received from the API

    central_client.execute_actions(action_list)
