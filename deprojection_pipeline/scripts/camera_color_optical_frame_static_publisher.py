import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
import os
import pickle

class CameraColorOpticalFrameStaticPublisher:
    def __init__(self)->None:
        self.config_path = os.path.join("/home", os.environ["USER"], 
                                        "catkin_ws/src/llm_manipulator_integration/deprojection_pipeline/config", 
                                        "camera_color_optical_frame_transformation.bin")
        
        self.config = self.load_config()
        self.static_broadcaster_camera_color_optical_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.map_base_link_broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.camera_color_optical_frame_base_link = TransformStamped()
        self.camera_color_optical_frame_base_link.header.stamp = rospy.Time.now()
        self.camera_color_optical_frame_base_link.header.frame_id = "camera_color_optical_frame"
        self.camera_color_optical_frame_base_link.child_frame_id = "base_link"
        self.camera_color_optical_frame_base_link.transform.translation.x = self.config["translation"][0]
        self.camera_color_optical_frame_base_link.transform.translation.y = self.config["translation"][1]
        self.camera_color_optical_frame_base_link.transform.translation.z = self.config["translation"][2]
        self.camera_color_optical_frame_base_link.transform.rotation.x = self.config["rotation"][0]
        self.camera_color_optical_frame_base_link.transform.rotation.y = self.config["rotation"][1]
        self.camera_color_optical_frame_base_link.transform.rotation.z = self.config["rotation"][2]
        self.camera_color_optical_frame_base_link.transform.rotation.w = self.config["rotation"][3]

        rospy.loginfo(f"\"camera_color_optical_frame\" to \"base_link\" transformation : {self.camera_color_optical_frame_base_link}")
        self.static_broadcaster_camera_color_optical_broadcaster.sendTransform(self.camera_color_optical_frame_base_link)

    def load_config(self):
        with open(self.config_path,"rb") as file:
            self.config = pickle.load(file)
            return self.config

if __name__ == "__main__":
    rospy.init_node("camera_color_optical_frame_static_publisher")
    static_publisher = CameraColorOpticalFrameStaticPublisher()
    rospy.spin()

