#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
import numpy as np
import os, sys
import image_geometry
import tf.transformations as tr
import tf, tf2_ros

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
        self.timer_red = rospy.Timer(rospy.Duration(0.5), self.publish_red_fiducial)
        self.timer_blue = rospy.Timer(rospy.Duration(0.5), self.publish_blue_fiducial)
        self.timer_yellow = rospy.Timer(rospy.Duration(0.5), self.publish_yellow_fiducial)

    def rgb_image_callback(self,msg:Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

    def depth_image_callback(self,msg:Image):
        self.depth_image = (self.bridge.imgmsg_to_cv2(msg,desired_encoding="passthrough"))

    def color_camera_info_callback(self,msg:Image):
        self.color_camera_info = msg
    
    def depth_camera_info_callback(self,msg:Image):
        self.depth_camera_info = msg

    def publish_red_fiducial(self,event):
        if self.red_fiducial is not None:
            self.red_pub.publish(self.red_fiducial)
    
    def publish_blue_fiducial(self,event):
        if self.blue_fiducial is not None:
            self.blue_pub.publish(self.blue_fiducial)

    def publish_yellow_fiducial(self,event):
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

    def find_red_fiducial(self):
        rgb_image = self.rgb_image

        # mask for red
        lower_red = np.array([0, 0, 160])
        upper_red = np.array([100, 100, 255])

        # find centroid of the red fiducial
        mask = cv2.inRange(rgb_image, lower_red, upper_red)
    
        M = cv2.moments(mask)
        if M["m00"] != 0:
            red_x = int(M["m10"] / M["m00"])
            red_y = int(M["m01"] / M["m00"])
            print("Centroid of red fiducial: ", red_x, red_y)
        else:
            print("Object not found")

        # draw a point on the centroid in the original image
        cv2.circle(rgb_image, (red_x, red_y), 2, (0, 0, 0), -1)

        # get the 3d position of the red fiducial
        self.camera_model.fromCameraInfo(self.depth_camera_info)
        red_position = self.get_3d_position(red_x, red_y)
        print("3D position of red fiducial: ", red_position)

        self.red_fiducial = red_position
        return red_position
    
    def find_blue_fiducial(self):
        rgb_image = self.rgb_image

        # mask for blue
        lower_blue = np.array([115, 70, 0])
        upper_blue = np.array([255, 150, 75])

        # find centroid of the blue fiducial
        mask = cv2.inRange(rgb_image, lower_blue, upper_blue)
    
        M = cv2.moments(mask)
        if M["m00"] != 0:
            blue_x = int(M["m10"] / M["m00"])
            blue_y = int(M["m01"] / M["m00"])
            print("Centroid of blue fiducial: ", blue_x, blue_y)
        else:
            print("Object not found")

        # draw a point on the centroid in the original image
        cv2.circle(rgb_image, (blue_x, blue_y), 2, (0, 0, 0), -1)

        # get the 3d position of the blue fiducial
        self.camera_model.fromCameraInfo(self.depth_camera_info)
        blue_position = self.get_3d_position(blue_x, blue_y)
        print("3D position of blue fiducial: ", blue_position)

        self.blue_fiducial = blue_position
        return blue_position
    
    def find_yellow_fiducial(self):
        rgb_image = self.rgb_image

        # mask for yellow
        lower_yellow = np.array([35, 150, 150])
        upper_yellow = np.array([80, 215, 215])

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
        cv2.circle(rgb_image, (yellow_x, yellow_y), 2, (0, 0, 0), -1)

        # get the 3d position of the yellow fiducial
        self.camera_model.fromCameraInfo(self.depth_camera_info)
        yellow_position = self.get_3d_position(yellow_x, yellow_y)
        print("3D position of yellow fiducial: ", yellow_position)

        self.yellow_fiducial = yellow_position
        return yellow_position

    def calibrate(self,robot_points,image_points):
        # Step 1: Compute the centroids
        centroid_A = np.mean(robot_points, axis=0)
        centroid_B = np.mean(image_points, axis=0)

        # Step 2: Subtract centroids
        A_centered = robot_points - centroid_A
        B_centered = image_points - centroid_B

        # Step 3: Compute the covariance matrix
        H = np.dot(A_centered.T, B_centered)

        # Step 4: Perform SVD
        U, S, Vt = np.linalg.svd(H)
        V = Vt.T

        # Step 5: Compute the rotation matrix
        R = np.dot(V, U.T)

        # Ensure a proper rotation matrix (no reflection)
        if np.linalg.det(R) < 0:
            V[:, -1] *= -1
            R = np.dot(V, U.T)

        # Step 6: Compute the translation vector
        t = centroid_B - np.dot(R, centroid_A)

        R = np.vstack((R, [0, 0, 0]))
        R = np.hstack((R, [[0], [0], [0], [1]]))
        q = tr.quaternion_from_matrix(R)
        return q,t


def find_transformation_matrix(A,B):
    # Step 1: Compute the centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # Step 2: Subtract centroids
    A_centered = A - centroid_A
    B_centered = B - centroid_B

    # Step 3: Compute the covariance matrix
    H = np.dot(A_centered.T, B_centered)

    # Step 4: Perform SVD
    U, S, Vt = np.linalg.svd(H)
    V = Vt.T

    # Step 5: Compute the rotation matrix
    R = np.dot(V, U.T)

    # Ensure a proper rotation matrix (no reflection)
    if np.linalg.det(R) < 0:
        V[:, -1] *= -1
        R = np.dot(V, U.T)

    # Step 6: Compute the translation vector
    t = centroid_B - np.dot(R, centroid_A)

    R = np.vstack((R, [0, 0, 0]))
    R = np.hstack((R, [[0], [0], [0], [1]]))
    q = tr.quaternion_from_matrix(R)
    return q,t

if __name__ == "__main__":
    rospy.init_node("camera_calibration")
    calibration = Calibration()
    rospy.sleep(0.1)

    robot_points = []
    camera_points = []

    # for i in range(4):
    #     print("Move the robot to the fiducial ", i+1)
    #     input("Press enter to capture the image")
    #     camera_points.append(calibration.find_red_fiducial().point)
        
    #     input("Presss enter to input robot coordinates : ")
    #     x = float(input("Enter the x coordinate of the robot: "))
    #     y = float(input("Enter the y coordinate of the robot: "))
    #     z = float(input("Enter the z coordinate of the robot: "))
    #     robot_points.append([x, y, z])

    camera_points = np.array([[-0.053038530884010324,-0.16344921950899796,1.0246913032145657],
                          [-0.15135917511246952,0.12394492240265864,0.8723325376941249],
                          [0.18097735075533558,0.06147174141413742,0.9050378022608193],
                          [0.013290463526259908,-0.0012266763565194055,0.9459058403691]
                          ])
    object_points = np.array([[-0.13562,-0.46403,-0.39650],
                          [-0.14516,-0.79955,-0.39650],
                          [0.16722,-0.64270,-0.39862],
                          [-0.01980,-0.61055,-0.39715]
                          ])
                          
    
    q,t = calibration.calibrate(robot_points=object_points, image_points=camera_points)
    print("Translation from robot frame to image frame = ", t)
    print("Quaternion from robot frame to image frame = ", q)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    T = TransformStamped()
    T.header.stamp = rospy.Time.now()
    T.header.frame_id = "camera_color_optical_frame"
    T.child_frame_id = "base_link"
    T.transform.translation.x = t[0]
    T.transform.translation.y = t[1]
    T.transform.translation.z = t[2]
    T.transform.rotation.x = q[0]
    T.transform.rotation.y = q[1]
    T.transform.rotation.z = q[2]
    T.transform.rotation.w = q[3]
    print("Sending Transform T here : ",T)
    broadcaster.sendTransform(T)

    rospy.spin()
    

    