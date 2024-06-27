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
import json
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
        
        # Service
        self.get_object_locations_service = rospy.Service(
                                                            "get_object_locations",
                                                            GetObjectLocations,
                                                            self.detect
        )
        
        ################ this is for testing ################                                         )
        self.orange_position_pub = rospy.Publisher("/orange_position", PointStamped, queue_size=10)
        self.orange_position = None
        self.timer_pub = rospy.Timer(rospy.Duration(0.5), self.publish_stream)
        #####################################################
        
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

    def get_3d_position(self, x, y):
        if self.depth_image is None and self.camera_info is None:
            return  # Wait until depth image is received
        depth = (self.depth_image[y, x])/1000  # Convert to meters
        if np.isnan(depth) or depth == 0:
            rospy.logwarn("Invalid depth at pixel ({}, {})".format(x, y))
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
        return position

    # service
    def detect(self,request):
        det_result = self.detection_model(self.color_image)
        
        # extract id, class, position from detection result
        class_names = det_result[0].names
        json_result = det_result[0].tojson()
        dtype_result = json.loads(json_result)

        result = ObjectPositions()
        index = 0

        for detection in dtype_result:
            if detection["confidence"] > 0.5:
                x = int((detection["box"]["x1"] + detection["box"]["x2"]) / 2)
                y = int((detection["box"]["y1"] + detection["box"]["y2"]) / 2)
                position = self.get_3d_position(x,y)
                if position is None:
                    return
                
                object_position = ObjectPosition()
                object_position.id = index
                object_position.Class = class_names[detection["class"]]
                object_position.position = position
                result.object_position.append(object_position)

                ########## this is for testing ###########
                if object_position.Class == "orange":
                    self.orange_position = position
                ##########################################

            index += 1
        return GetObjectLocationsResponse(result)

    ########## this is for testing ###########
    def publish_stream(self,event):
        if self.orange_position is not None:
            self.orange_position_pub.publish(self.orange_position)
    ##########################################


if __name__ == "__main__":
    rospy.init_node("ultralytics_node")
    ultralytics_instance = Ultralytics()
    time.sleep(0.1)
    rospy.spin()
