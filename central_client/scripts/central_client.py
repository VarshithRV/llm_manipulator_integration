import rospy
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests

class CentralClient:
    def __init__(self) -> None:
        self.get_object_locations_service = rospy.ServiceProxy(
            "get_object_locations",
            GetObjectLocations
        )

    def get_object_locations(self):
        try:
            response = self.get_object_locations_service()
            return response
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def convert_image_to_text(self,image: Image) -> str:
    # This is also how OpenAI encodes images: https://platform.openai.com/docs/guides/vision
        with io.BytesIO() as output:
            image.save(output, format="PNG")
            data = output.getvalue()
        return base64.b64encode(data).decode("utf-8")
    
    def test_demo_api(
        self,
        instruction: str,
        response: GetObjectLocationsResponse,
        url: str = "https://handsomely-lying-octagon.glitch.me/generate_plan",
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

        image = {
            "base64_string": self.convert_image_to_text(image_),
            "objects": objects
        }

        data = {
            "instruction": instruction,
            "images": [image],
        }

        headers = {"Content-Type": "application/json"}
        response = requests.post(url, data=json.dumps(data), headers=headers)
        return response.json()
    
    # execute all the actions in the action list one by one here.
    def execute_actions(self, action_list):
        print("Executing actions ...")
        for action in action_list:
            print("Executing action : ", action["action_type"])
            print("Source object position : ")
            print(action["source_object_position"])
            print("Target object position : ") 
            print(action["target_object_position"])
            rospy.sleep(1)



if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    # Call the service, response contains object_positions, bouding_boxes and image
    response = central_client.get_object_locations()

    rospy.sleep(0.5)
    # instruction = "pick up the orange object"
    instruction = "pick up all the oranges and place it in a bowl"
    print("Requesting for plan ...")
    print("Prompt : ", instruction)

    plan = central_client.test_demo_api(instruction, response)
    # contains the list of actions
    # action {"action_type":"pick_and_place", "source_object_id":"4", "target_object_id":"5"}
    # print(plan["plan_actions"])
    for action in plan["plan_actions"]:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])

    action_list = []
    for action in plan["plan_actions"]:
        action_parsed = {}
        action_parsed["action_type"] = action["action_type"]
        action_parsed["source_object_position"] = response.result.object_position[action["source_object_id"]].position
        action_parsed["target_object_position"] = response.result.object_position[action["target_object_id"]].position
        action_list.append(action_parsed)

    central_client.execute_actions(action_list)
