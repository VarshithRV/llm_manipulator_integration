#!/usr/env/bin python3

import rospy
from calibration import Calibration
import geometry_msgs.msg
from tf2_geometry_msgs import PointStamped
import tf2_ros


class CalibrationValidation(Calibration):

    def __init__(self) -> None:
        super().__init__()
        rospy.sleep(0.1)
        self.red_fiducial_base_link_position = None
        self.red_fiducial_camera_frame_position = self.find_red_fiducial()
        print(self.red_fiducial_camera_frame_position)
        self.transform_buffer = tf2_ros.Buffer()
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)
        rospy.sleep(0.1)
        self.red_fiducial_base_link_position_pub = rospy.Publisher("/red_fiducial_base_link_position", geometry_msgs.msg.PointStamped, queue_size=10)
        self.publish_timer = rospy.Timer(rospy.Duration(0.5), self.publish_red_fiducial_base_link_position)
        self.transform_position()

    def publish_red_fiducial_base_link_position(self, event):
        if self.red_fiducial_base_link_position is not None:
            self.red_fiducial_base_link_position_pub.publish(self.red_fiducial_base_link_position)
        else:
            pass

    def transform_position(self):
        try:
            self.red_fiducial_base_link_position = self.transform_buffer.transform(self.red_fiducial_camera_frame_position, "base_link")
            rospy.loginfo(f"Red fiducial position in base_link frame : {self.red_fiducial_base_link_position}")
            return self.red_fiducial_base_link_position
        except Exception as e:
            rospy.logerr(e)
            return None
        


if __name__ == "__main__":
    rospy.init_node("calibration_validation")
    calibration_validation = CalibrationValidation()
    rospy.spin()