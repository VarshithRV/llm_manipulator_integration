#!/usr/bin/env/ python3

import rospy
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PointStamped
import cv_bridge
import image_geometry
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from deprojection_pipeline.msg import ObjectPosition, ObjectPositions
from deprojection_pipeline.srv import GetObjectLocations, GetObjectLocationsResponse

####### probability threshold ########
THRESHOLD = 0.5
######################################

class Deprojection:

    def __init__(self) -> None:
        
        self.bounding_boxes =  None # contains a list of bounding box, with probability > THRESHOLD at the instant
        self.depth_image = None # contains the depth images
        self.camera_info = None # contains the camera info
        self.color_image = None # contains the color image
        self.camera_model = image_geometry.PinholeCameraModel()
        self.cv_bridge = cv_bridge.CvBridge()

        # Subscribers
        self.bounding_boxes_sub = rospy.Subscriber(
                                                   "/darknet_ros/bounding_boxes", 
                                                   BoundingBoxes, 
                                                   self.bounding_boxes_callback
                                                   )
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
        server = rospy.Service("get_object_locations", GetObjectLocations, self.get_object_locations)
        pass


    def get_3d_position(self, x, y):
        if self.depth_image is None:
            return  # Wait until depth image is received
        depth = (self.depth_image[x, y])/1000  # Convert to meters
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
        

    # this is the server function
    def get_object_locations(self,request):
        result = ObjectPositions()
        for box in self.bounding_boxes:
            object_position = ObjectPosition()
            id_ = box.id
            class_ = box.Class
            x = int((box.xmin + box.xmax) /2)
            y = int((box.ymin + box.ymax) /2)
            position = self.get_3d_position(x, y)
            object_position.id = id_
            object_position.Class = class_
            object_position.position = position
            result.object_position.append(object_position)
        
        # return result
        return GetObjectLocationsResponse(result)

    
    def bounding_boxes_callback(self, msg: BoundingBoxes):
        self.bounding_boxes = list()
        for box in msg.bounding_boxes:
            if box.probability > THRESHOLD:
                self.bounding_boxes.append(box)
                break
        pass
    
    def color_image_callback(self, msg: Image):
        self.color_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pass
    
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        self.camera_model.fromCameraInfo(msg)
        pass


if __name__ == "__main__":
    rospy.init_node("deprojection_node")
    deproject = Deprojection()
    rospy.sleep(0.5)
    rospy.loginfo("Deproject server ready...")
    rospy.spin()

