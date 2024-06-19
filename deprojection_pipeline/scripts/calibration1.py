#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import numpy as np
import os, sys
import image_geometry


##################### FIDUCIAL POSITION WITH RESPECT TO THE ROBOT BASE LINK ############################
RED_X = 0.1
RED_Y = 0.1
RED_Z = 0.1

BLUE_X = 0.1
BLUE_Y = 0.1
BLUE_Z = 0.1

YELLOW_X = 0.1
YELLOW_Y = 0.1
YELLOW_Z = 0.1
#########################################################################################################



class Calibration:
    def __init__(self)->None:
        self.rgb_image = None
        self.depth_image = None
        self.depth_camera_info = None
        self.color_camera_info = None
        self.camera_model = image_geometry.PinholeCameraModel()
        self.bridge = cv_bridge.CvBridge()
        self.red_fiducial = None
        self.blue_fiducial = None
        self.yellow_fiducial = None

        self.rgb_image_subscriber = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            callback=self.rgb_image_callback
        )
        self.depth_image_subscriber = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw",
            Image,
            callback=self.depth_image_callback
        )
        self.depth_camera_info_subscriber = rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            callback=self.depth_camera_info_callback
        )
        self.color_camera_info_subscriber = rospy.Subscriber(
            "/camera/color/camera_info",
            CameraInfo,
            callback=self.color_camera_info_callback
        )
        

        # fiducial publishers
        self.red_pub = rospy.Publisher("/fiducial/red", PointStamped, queue_size=10)
        self.blue_pub = rospy.Publisher("/fiducial/blue", PointStamped, queue_size=10)
        self.yellow_pub = rospy.Publisher("/fiducial/yellow", PointStamped, queue_size=10)

        # create a timer to publish fiducials
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish_fiducial)


    def rgb_image_callback(self,msg:Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def depth_image_callback(self,msg:Image):
        self.depth_image = (self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough"))

    def color_camera_info_callback(self,msg:Image):
        self.color_camera_info = msg
    
    def depth_camera_info_callback(self,msg:Image):
        self.depth_camera_info = msg

    def publish_fiducial(self,event):
        if self.red_fiducial is not None:
            self.red_pub.publish(self.red_fiducial)
        if self.blue_fiducial is not None:
            self.blue_pub.publish(self.blue_fiducial)
        if self.yellow_fiducial is not None:
            self.yellow_pub.publish(self.yellow_fiducial)

    def get_3d_position(self, x, y):
        if self.depth_image is None and self.depth_camera_info is None:
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

    def find_positions(self):
        rgb_image = self.rgb_image

        # mask for red
        lower_red = np.array([0, 0, 160])
        upper_red = np.array([100, 100, 255])
        
        # mask for blue
        lower_blue = np.array([115, 70, 0])
        upper_blue = np.array([255, 150, 75])

        # mask for yellow
        lower_yellow = np.array([35, 150, 150])
        upper_yellow = np.array([80, 215, 215])

        # find centroid of the red fiducial
        mask = cv2.inRange(rgb_image, lower_red, upper_red)
    
        M = cv2.moments(mask)
        if M["m00"] != 0:
            red_x = int(M["m10"] / M["m00"])
            red_y = int(M["m01"] / M["m00"])
            print("Centroid of red fiducial: ", red_x, red_y)
        else:
            print("Object not found")

        # find centroid of the blue fiducial
        mask = cv2.inRange(rgb_image, lower_blue, upper_blue)
    
        M = cv2.moments(mask)
        if M["m00"] != 0:
            blue_x = int(M["m10"] / M["m00"])
            blue_y = int(M["m01"] / M["m00"])
            print("Centroid of blue fiducial: ", blue_x, blue_y)
        else:
            print("Object not found")

        # find centroid of the yellow fiducial
        mask = cv2.inRange(rgb_image, lower_yellow, upper_yellow)
    
        M = cv2.moments(mask)
        if M["m00"] != 0:
            yellow_x = int(M["m10"] / M["m00"])
            yellow_y = int(M["m01"] / M["m00"])
            print("Centroid of yellow fiducial: ", yellow_x, yellow_y)
        else:
            print("Object not found")

        # draw a point on the centroid in the original image
        cv2.circle(rgb_image, (red_x, red_y), 2, (0, 0, 0), -1)
        cv2.circle(rgb_image, (blue_x, blue_y), 2, (0, 0, 0), -1)
        cv2.circle(rgb_image, (yellow_x, yellow_y), 2, (0, 0, 0), -1)
        

        # get the 3d positions of all the fiducials
        self.camera_model.fromCameraInfo(self.depth_camera_info)
        red_position = self.get_3d_position(red_x, red_y)
        blue_position = self.get_3d_position(blue_x, blue_y)
        yellow_position = self.get_3d_position(yellow_x, yellow_y)
        
        print("3D position of red fiducial: ", red_position)
        print("3D position of blue fiducial: ", blue_position)
        print("3D position of yellow fiducial: ", yellow_position)

        self.red_fiducial = red_position
        self.blue_fiducial = blue_position
        self.yellow_fiducial = yellow_position

    def calibrate(self):
        if self.red_fiducial is None or self.blue_fiducial is None or self.yellow_fiducial is None:
            rospy.logwarn("Fiducials not found")
            return

        print("Calibrating the camera position ...")

        # solvepnp to get the transformation matrix
        robot_points = np.array([
            [RED_X, RED_Y, RED_Z],
            [BLUE_X, BLUE_Y, BLUE_Z],
            [YELLOW_X, YELLOW_Y, YELLOW_Z]
        ], dtype=np.float32)

        image_points = np.array([
            [self.red_fiducial.point.x, self.red_fiducial.point.y, self.red_fiducial.point.z],
            [self.blue_fiducial.point.x, self.blue_fiducial.point.y, self.blue_fiducial.point.z],
            [self.yellow_fiducial.point.x, self.yellow_fiducial.point.y, self.yellow_fiducial.point.z]
        ], dtype=np.float32)

        # camera matrix
        camera_matrix = np.array([
            [self.depth_camera_info.K[0], self.depth_camera_info.K[1], self.depth_camera_info.K[2]],
            [self.depth_camera_info.K[3], self.depth_camera_info.K[4], self.depth_camera_info.K[5]],
            [self.depth_camera_info.K[6], self.depth_camera_info.K[7], self.depth_camera_info.K[8]]
        ], dtype=np.float32)

        dist_coeffs = np.array([0, 0, 0, 0], dtype=np.float32)

        # solvepnp
        retval, rvec, tvec = cv2.solvePnP(robot_points, image_points, camera_matrix, dist_coeffs)

        # get the rotation matrix
        R, _ = cv2.Rodrigues(rvec)

        # get the transformation matrix
        T = np.hstack((R, tvec))

        # add a row to the transformation matrix
        T = np.vstack((T, [0, 0, 0, 1]))

        print("Transformation matrix: ", T)


if __name__ == "__main__":
    rospy.init_node("camera_calibration")
    calibration = Calibration()
    rospy.sleep(0.5)
    calibration.find_positions()
    # calibration.calibrate()
    rospy.spin()
    

    