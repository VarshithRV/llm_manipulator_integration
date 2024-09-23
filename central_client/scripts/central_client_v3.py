import rospy
from deprojection_pipeline_msgs.srv import GetObjectLocations, GetObjectLocationsResponse
import cv_bridge
import cv2
import sensor_msgs.msg
from PIL import Image, ImageDraw
import numpy as np
import random
import io
import base64
import json
import requests
from geometry_msgs.msg import PointStamped
from motion_planning_server_msgs.msg import PickPlaceAction, PickPlaceGoal, PickPlaceResult
from motion_planning_server_msgs.msg import MovePreactionAction, MovePreactionActionGoal, MovePreactionActionResult
import actionlib
import tf2_ros
from tf2_geometry_msgs import PointStamped
import image_geometry
import  threading
import speech_recognition as sr
from deep_translator import GoogleTranslator
from gtts import gTTS
import pygame
import io
import sys


class CentralClient:
    def __init__(self) -> None:

        self.depth_image = None # contains the depth  cv images
        self.camera_info = None # contains the camera info
        self.color_image = None # contains the color cv image
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        # transform listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # action client for pick and place
        self.pick_place_client = actionlib.SimpleActionClient("pick_place", PickPlaceAction)
        # action client for moving to preaction position
        self.move_preaction_client = actionlib.SimpleActionClient("move_preaction", MovePreactionAction)
        # action client for cleaning
        self.clean_client = actionlib.SimpleActionClient("clean", PickPlaceAction)
        self.clean_client.wait_for_server()
        self.pick_place_client.wait_for_server()
        self.move_preaction_client.wait_for_server()

        # Subscriber for realsense camera
        self.depth_image_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", 
            sensor_msgs.msg.Image, 
            self.depth_image_callback
        )

        self.camera_info_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info", 
            sensor_msgs.msg.CameraInfo, 
            self.camera_info_callback
        )

        self.color_image_sub = rospy.Subscriber(
            "/camera/color/image_raw",
            sensor_msgs.msg.Image,
            self.color_image_callback
        )


    def convert_image_to_text(self,image: Image) -> str:
    # This is also how OpenAI encodes images: https://platform.openai.com/docs/guides/vision
        with io.BytesIO() as output:
            image.save(output, format="PNG")
            data = output.getvalue()
        return base64.b64encode(data).decode("utf-8")
    
    def get_plan(
        self,
        url: str = "https://robot-api-2.glitch.me/handle_request",
        instruction: str = "Put the objects in the bowl in order of most sweet to least sweet.",
        job_type: str = "grounding_and_planning",
    ):
        
        if self.color_image is None:
            rospy.logerr("Color image not received yet")
            return
        
        image = self.color_image
        image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        data = {
            "instruction": instruction,
            "images": [dict(base64_string=self.convert_image_to_text(image), objects=[])],
        }
        headers = {"Content-Type": "application/json"}
        content = dict(data=data, job_type=job_type)
        response = (requests.post(url, data=json.dumps(content), headers=headers)).json()

        return response["objects"], response["plan_actions"], response["raw_output"]
        
        # the response.json() contains the objects(id, label, box), the plan_action(action list)
        # and the raw_output(explanation)

    # execute all the actions in the action list one by one here.
    def execute_actions(self, action_list):
        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.move_preaction_client.send_goal(move_preaction_goal)
        self.move_preaction_client.wait_for_result()
        move_preaction_result = self.move_preaction_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        
        clean_goal = PickPlaceGoal()
        clean_goal.source = PointStamped()
        clean_goal.destination = PointStamped()
        print("Cleansing the Gripper")
        self.clean_client.send_goal(clean_goal)
        self.clean_client.wait_for_result()
        clean_result = self.clean_client.get_result()
        print("clean result : ", clean_result.result)
        
        print("Executing actions ...")
        
        for action in action_list:
            source = action["source_object_position"]
            destination = action["target_object_position"]

            ### this is for testing ###
            source.point.z += 0
            destination.point.z += 0

            rospy.loginfo("Sending pick and place goal")
            pick_place_goal = PickPlaceGoal()
            pick_place_goal.source = source
            pick_place_goal.destination = destination
            self.pick_place_client.send_goal(pick_place_goal)
            self.pick_place_client.wait_for_result()
            pick_place_result = self.pick_place_client.get_result()
            print("Pick and place result : ", pick_place_result.result)
            rospy.sleep(0.3)
            # x = input("Press Enter to continue")

        clean_goal = PickPlaceGoal()
        clean_goal.source = PointStamped()
        clean_goal.destination = PointStamped()
        print("Cleansing the Gripper")
        self.clean_client.send_goal(clean_goal)
        self.clean_client.wait_for_result()
        clean_result = self.clean_client.get_result()
        print("clean result : ", clean_result.result)

        rospy.loginfo("Sending move preaction goal")
        move_preaction_goal = MovePreactionActionGoal()
        self.move_preaction_client.send_goal(move_preaction_goal)
        self.move_preaction_client.wait_for_result()
        move_preaction_result = self.move_preaction_client.get_result()
        print("Move preaction result : ", move_preaction_result.result)
        print("Done")

    def transform_position(self, source_position):
        try:
            tf_position = self.tf_buffer.transform(source_position, "base_link")
            # rospy.loginfo(f"Object position in base_link frame : {self.red_fiducial_base_link_position}")
            return tf_position
        except Exception as e:
            rospy.logerr(e)
            return None

    def get_3d_position(self, x, y):
        if self.depth_image is None and self.camera_info is None:
            return  # Wait until depth image is received
        depth = (self.depth_image[y, x])/1000  # Convert to meters
        if np.isnan(depth) or depth == 0:
            rospy.logwarn("Invalid depth at pixel ({}, {})".format(x, y))
            ########## this is for debugging ###########
            mask = self.depth_image != 0
            mask = mask.astype(np.uint8)*255
            # draw a orange circle at the invalid depth pixel
            cv2.circle(mask, (x, y), 5, (255, 165, 0), -1)
            cv2.imwrite("invalid depth.jpg", mask)
            ############################################
            return
        
        # Project the 2D pixel to 3D point in the camera frame
        point_3d = self.camera_model.projectPixelTo3dRay((x, y))
        point_3d = np.array(point_3d) * depth  # Scale the ray by the depth
        position = PointStamped()
        position.header.frame_id = self.camera_model.tf_frame
        position.header.stamp = rospy.Time.now()
        position.point.x = point_3d[0]
        position.point.y = point_3d[1]
        position.point.z = point_3d[2]
        
        # transform the position to base_link frame
        position = self.transform_position(position)
        return position
    
    def color_image_callback(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        pass

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass
    
    def camera_info_callback(self, msg: sensor_msgs.msg.CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)
        pass

def text_to_speech(text, lang='en'):
    """将文本转换为语音并直接播放"""
    tts = gTTS(text=text, lang=lang, slow=False)  # 创建gTTS对象
    mp3_fp = io.BytesIO()
    tts.write_to_fp(mp3_fp)  # 写入内存文件
    mp3_fp.seek(0)           # 移动到文件的开始

    pygame.mixer.init()      # 初始化pygame混音器
    pygame.mixer.music.load(mp3_fp)  # 加载内存中的音频文件
    pygame.mixer.music.play()        # 播放音频
    while pygame.mixer.music.get_busy():  # 检查音乐流是否正在播放
        pygame.time.Clock().tick(10)  # 等待播放结束
    

def speech_to_text():
    # 初始化识别器
    recognizer = sr.Recognizer()

    # 使用默认的麦克风
    with sr.Microphone() as source:
        recognizer.adjust_for_ambient_noise(source)
        print("Listening for speech...")
        print("Please speak something...")
        audio = recognizer.listen(source)

        # 尝试使用不同的语言进行识别
        languages = ['en-US']  # 英语、简体中文
        # languages = ['zh-CN']
        text_result=""
        for lang in languages:
            try:
                # 使用 Google 的语音识别服务
                text = recognizer.recognize_google(audio, language=lang)
                print(f"Detected ({lang}): " + text)
                text_result=text
                if lang != 'en-US':
                    # 翻译成英语
                    text = GoogleTranslator(source='auto', target='en').translate(text)
                    print("Translated to English: " + text)
                    text_result=text
            except sr.UnknownValueError:
                print(f"Google Speech Recognition could not understand audio in {lang}")
            except sr.RequestError as e:
                print(f"Could not request results from Google Speech Recognition service in {lang}; {e}")
    return text_result

# driver for the central client
if __name__ == "__main__":
    rospy.init_node("central_client")
    central_client = CentralClient()
    rospy.sleep(0.1)
    
    # get instruction from the user
    # instruction = input("Enter the prompt : ")
    instruction = speech_to_text()
    print("Requesting for plan from API ...")
    print("Prompt : ", instruction)
    print("Getting plan ...")

    # get plan, 2d object positions, raw output
    objects, plan_actions, raw_output = central_client.get_plan(instruction=instruction)

    # validate the response
    if (objects is None or plan_actions is None or 
        raw_output is None or len(objects) == 0 or 
        len(plan_actions) == 0 or raw_output == ""):
        rospy.logerr("API response is None")
        exit()
    
    # display the output
    print("Objects detected : ", len(objects))
    for object in objects:
        print(object["label"])
    for plan in plan_actions:
        print(plan)
    print("Explanation : ",raw_output)

    # create a thread
    thread = threading.Thread(target=text_to_speech, args=(raw_output,))
    thread.start()
    
    # display the image with the objects and the bounding boxes
    image = Image.fromarray(
        cv2.cvtColor(
            central_client.color_image, cv2.COLOR_BGR2RGB
            )
        )
    
    draw = ImageDraw.Draw(image)
    
    for object in objects:
        draw.rectangle(object["box"], outline="red", width=2)
        position = (int(object["box"][0]), int(object["box"][1]) - min(image.size) // 32)
        draw.rectangle(draw.textbbox(position, object["label"]), fill="red")
        draw.text(position, object["label"], fill="white")

    image.save("bounding boxes.png")


    # process plan actions and compute the 3d positions
    print("Computing positions ..... ")
    
    objects_3d = []
    for object in objects:
        object_3d = {}
        x = int((object["box"][0] + object["box"][2])//2)
        y = int((object["box"][1] + object["box"][3])//2)
        position = PointStamped()
        position = central_client.get_3d_position(x, y)
        label = object["label"]
        label = label.split(" ")
        object_3d["id"] = int(label[0])
        # object_3d["label"] = label[1] + " " + label[2]
        # loop through the label to get the full label
        object_3d["label"] = ""
        for i in range(1, len(label)):
            object_3d["label"] += label[i] + " "
        object_3d["position"] = position
        
        # label generic position adjustments
        object_3d["position"].point.x += 0.01
        object_3d["position"].point.y += 0.01
        object_3d["position"].point.z -= 0.00

        print("object : ", object_3d["label"])
        #  label specific position adjustments
        if object_3d["label"] == "green beans bowl ":
            object_3d["position"].point.z -= 0.01
        if object_3d["label"] == "purple onions bowl ":
            object_3d["position"].point.z -= 0.017
        objects_3d.append(object_3d)

    for action in plan_actions:
        action["source_object_id"] = int(action["source_object_id"])
        action["target_object_id"] = int(action["target_object_id"])
    
    # variable formats : 
    # object_3d = {"id":1, "label":"apple", "position":position:PointStamped}
    # action = {"action_type":"pick_and_place", "source_object_id":1, "target_object_id":2}

    action_list = []
    for action in plan_actions :
        action_parsed = {
            "action_type": action["action_type"],
            "source_object_position": objects_3d[action["source_object_id"]]["position"],
            "target_object_position": objects_3d[action["target_object_id"]]["position"]
        }
        action_list.append(action_parsed)

    # user validation of the action list and proceed to execute the actions
    input("Press Enter to continue ...")
    
    # execution of the actions
    central_client.execute_actions(action_list=action_list)
    thread.join()
