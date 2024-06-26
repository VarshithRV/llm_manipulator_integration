#!/usr/bin/env/ python3
import time
import rospy
from ultralytics import YOLO
import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
import cv_bridge
import image_geometry
import cv2
import numpy as np
from deprojection_pipeline.msg import ObjectPosition, ObjectPositions
from deprojection_pipeline.srv import GetObjectLocations, GetObjectLocationsResponse

class Ultralytics:
    def __init__(self) -> None:
        self.detection_model = YOLO("yolov8m.pt")
        self.segmentation_model = YOLO("yolov8m-seg.pt")
        self.depth_image = None # contains the depth  cv images
        self.camera_info = None # contains the camera info
        self.color_image = None # contains the color cv image
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        # Subscribers
        self.depth_image_sub = rospy.Subscriber(
                                                "/camera/aligned_depth_to_color/image_raw", 
                                                Image, 
                                                self.depth_image_callback
                                                )
        self.camera_info_sub = rospy.Subscriber(
                                                "/camera/aligned_depth_to_color/camera_info", 
                                                CameraInfo, 
                                                self.camera_info_callback
                                                )
        self.color_image_sub = rospy.Subscriber(
                                                "/camera/color/image_raw",
                                                Image,
                                                self.color_image_callback
                                                )
        
        
    def color_image_callback(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        pass

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass
    
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)
        pass

    def detect(self):
        det_result = self.detection_model(self.color_image)
        det_annotated = det_result[0].plot(show=False)
        return det_annotated

if __name__ == "__main__":
    rospy.init_node("ultralytics_node")
    ultralytics_instance = Ultralytics()
    time.sleep(0.5)
    det_annotated = ultralytics_instance.detect()
    cv2.imshow("detected", det_annotated)
    cv2.waitKey(-1)