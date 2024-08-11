#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PointStamped, PoseStamped, Pose
from my_moveit_planner.action import FinalAction, FinalGoal
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
from PIL import Image
import io
import base64
import json
import requests
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_commander.exception import MoveItCommanderException

class CentralClient:
    def __init__(self):
        self.get_object_locations_service = rospy.ServiceProxy("get_object_locations", GetObjectLocations)
        self.action_client = actionlib.SimpleActionClient('task_server', FinalAction)

        # Initialize MoveIt! commander
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_move_group = MoveGroupCommander("manipulator")
        self.arm_move_group.set_planning_time(20.0)  

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
    
    def test_demo_api(self, instruction: str, response: GetObjectLocationsResponse, url: str = "https://handsomely-lying-octagon.glitch.me/generate_plan"):
        image_ = cv_bridge.CvBridge().imgmsg_to_cv2(response.result.image, desired_encoding="bgr8")
        image_ = Image.fromarray(cv2.cvtColor(image_, cv2.COLOR_BGR2RGB))

        # create a timestamp using ros
        timestamp = rospy.Time.now()
        image_.save(str(timestamp)+".png")

        objects = []
        for object_position in response.result.object_position:
            objects.append({
                "object_id": object_position.id,
                "x_min": object_position.x_min,
                "y_min": object_position.y_min,
                "x_max": object_position.x_max,
                "y_max": object_position.y_max,
            })
        
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
    
    def send_goal(self, start_pose, end_pose):
        self.action_client.wait_for_server()
        goal = FinalGoal()
        goal.start_pose = start_pose
        goal.end_pose = end_pose

        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()

    def plan_and_execute(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = "base_link"

        self.arm_move_group.set_pose_target(pose_stamped)

        try:
            plan = self.arm_move_group.plan()
            if isinstance(plan, tuple):
                plan = plan[1]
                
            if not plan or len(plan.joint_trajectory.points) == 0:
                rospy.logerr("Planning failed")
                return False

            rospy.loginfo("Executing plan")
            success = self.arm_move_group.execute(plan, wait=True)
            if not success:
                rospy.logerr("Execution failed")
                return False

            self.arm_move_group.stop()
            self.arm_move_group.clear_pose_targets()
            return True
        except MoveItCommanderException as e:
            rospy.logerr("MoveItCommanderException: %s" % str(e))
            return False

    def execute_actions(self, action_list):
        print("Executing actions ...")
        for action in action_list:
            print("Executing action : ", action["action_type"])
            print("Source object position : ")
            print(action["source_object_position"])
            print("Target object position : ") 
            print(action["target_object_position"])

            start_pose = Pose()
            start_pose.position.x = action["source_object_position"].point.x + 0.02
            start_pose.position.y = action["source_object_position"].point.y
            start_pose.position.z = action["source_object_position"].point.z + 0.120
            start_pose.orientation.w = 1.0

            end_pose = Pose()
            end_pose.position.x = action["target_object_position"].point.x  + 0.02
            end_pose.position.y = action["target_object_position"].point.y
            end_pose.position.z = action["target_object_position"].point.z  + 0.120
            end_pose.orientation.w = 1.0

            # Plan and execute to the start pose
            if not self.plan_and_execute(start_pose):
                print("Action failed")
                break

            result = self.send_goal(PointStamped(point=start_pose.position), PointStamped(point=end_pose.position))
            if not result.success:
                print("Action failed")
                break

            # Plan and execute to the end pose
            if not self.plan_and_execute(end_pose):
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

    rospy.sleep(0.5)
    instruction = "pick up all the oranges and place it in a bowl"
    print("Requesting for plan ...")
    print("Prompt : ", instruction)

    plan = central_client.test_demo_api(instruction, response)
    for action in plan["plan_actions"]:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])

    action_list = []
    for action in plan["plan_actions"]:
        action_parsed = {
            "action_type": action["action_type"],
            "source_object_position": response.result.object_position[action["source_object_id"]].position,
            "target_object_position": response.result.object_position[action["target_object_id"]].position
        }
        action_list.append(action_parsed)

    central_client.execute_actions(action_list)
