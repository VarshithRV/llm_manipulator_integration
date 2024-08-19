import rospy
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests
from geometry_msgs.msg import PointStamped
from motion_planning_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from motion_planning_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib



class CentralClient:
    def __init__(self) -> None:
        self.get_object_locations_service = rospy.ServiceProxy(
            "get_object_locations",
            GetObjectLocations
        )
        self.pick_place_client = actionlib.SimpleActionClient("pick_place", PickPlaceAction)
        self.move_preaction_client = actionlib.SimpleActionClient("move_preaction", MovePreactionAction)
        self.pick_place_client.wait_for_server()
        self.move_preaction_client.wait_for_server()

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
        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.move_preaction_client.send_goal(move_preaction_goal)
        self.move_preaction_client.wait_for_result()
        move_preaction_result = self.move_preaction_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Executing actions ...")
        
        for action in action_list:
            source = action["source_object_position"]
            destination = action["target_object_position"]
            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
            self.pick_place_client.send_goal(pick_place_goal)
            self.pick_place_client.wait_for_result()
            pick_place_result = self.pick_place_client.get_result()
            print("Pick and place result : ", pick_place_result.result)
            rospy.sleep(1)

        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.move_preaction_client.send_goal(move_preaction_goal)
        self.move_preaction_client.wait_for_result()
        move_preaction_result = self.move_preaction_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)

        print("Done")

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
    
    
    # display the plan
    print(plan["plan_actions"])
    print("Explanation : ",plan["raw_output"])
    input("Press Enter to continue ...")
    
    # contains the list of actions
    # action {"action_type":"pick_and_place", "source_object_id":"4", "target_object_id":"5"}
    for action in plan["plan_actions"]:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])

    # prints the raw action list received from the API
    print(plan["plan_actions"])

    # substituting the object id with the object position
    action_list = []
    for action in plan["plan_actions"]:
        # get the class of the object for the source object
        source_object_class = response.result.object_position[action["source_object_id"]].Class
        if source_object_class == "orange":
            response.result.object_position[action["source_object_id"]].position.point.z -= 0.02
        if source_object_class == "carrot":
            response.result.object_position[action["source_object_id"]].position.point.z -= 0.02
        if source_object_class == "apple":
            response.result.object_position[action["source_object_id"]].position.point.z = response.result.object_position[action["source_object_id"]].position.point.z - 0.015
            response.result.object_position[action["source_object_id"]].position.point.x -= 0
            response.result.object_position[action["source_object_id"]].position.point.y += 0.02
        
        action_parsed = {
            "action_type": action["action_type"],
            "source_object_position": response.result.object_position[action["source_object_id"]].position,
            "target_object_position": response.result.object_position[action["target_object_id"]].position
        }
        action_list.append(action_parsed)


    # execution of the actions
    central_client.execute_actions(action_list)
