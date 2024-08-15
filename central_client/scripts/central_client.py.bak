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
    
    # execute all the actions in the action list one by one here.
    def execute_actions(self, action_list):
        print("Executing actions ...")
        # for action in action_list:
        #     print("Executing action : ", action["action_type"])
        #     print("Source object position : ")
        #     print(action["source_object_position"])
        #     print("Target object position : ") 
        #     print(action["target_object_position"])
        #     rospy.sleep(1)
        # print(action_list)

        # here there should be a client for our action


if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    time = rospy.Time.now()
    # Call the service, response contains object_positions, bouding_boxes and image
    response = central_client.get_object_locations()
    # printing the object id and corresponding classes
    for object_thing in response.result.object_position:
        print("Object ID : ", object_thing.id)
        print("Object Class : ", object_thing.Class)


    print("Objects detected in time : ", rospy.Time.to_sec(rospy.Time.now()-time))
    time = rospy.Time.now()
    
    
    # calling the API to get the plan
    # print("Enter the prompt :")
    instruction = input("Enter the prompt : ")
    # instruction = "Pick the apple and put it in the bowl"
    print("Requesting for plan from API ...")
    print("Prompt : ", instruction)
    plan = central_client.test_demo_api(instruction, response)
    print("API finished in time : ", rospy.Time.to_sec(rospy.Time.now()-time))
    
    
    # contains the list of actions
    # action {"action_type":"pick_and_place", "source_object_id":"4", "target_object_id":"5"}
    for action in plan["plan_actions"]:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])

    # prints the raw action list received from the API
    print(plan["plan_actions"])

    # substitting the object id with the object position
    action_list = []
    for action in plan["plan_actions"]:
        action_parsed = {}
        action_parsed["action_type"] = action["action_type"]
        action_parsed["source_object_position"] = response.result.object_position[action["source_object_id"]].position
        action_parsed["target_object_position"] = response.result.object_position[action["target_object_id"]].position
        action_list.append(action_parsed)


    # simulate execution of the actions
    central_client.execute_actions(action_list)
