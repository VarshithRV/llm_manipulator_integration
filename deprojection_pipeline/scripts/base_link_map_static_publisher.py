import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros

class MapFrameStaticPublisher:
    def __init__(self)->None:
        self.map_base_link_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        self.map_base_link = TransformStamped()
        self.map_base_link.header.stamp = rospy.Time(0)
        self.map_base_link.header.frame_id = "base_link"
        self.map_base_link.child_frame_id = "map"
        self.map_base_link.transform.translation.x = 0.19645
        self.map_base_link.transform.translation.y = -0.62519
        self.map_base_link.transform.translation.z = -0.39773
        self.map_base_link.transform.rotation.x = 0
        self.map_base_link.transform.rotation.y = 0
        self.map_base_link.transform.rotation.z = 0
        self.map_base_link.transform.rotation.w = 1

        rospy.loginfo(f"\"base_link\" to \"map\" transformation : {self.map_base_link}")
        self.map_base_link_broadcaster.sendTransform(self.map_base_link)

if __name__ == "__main__":
    rospy.init_node("map_frame_static_publisher")
    static_publisher = MapFrameStaticPublisher()
    rospy.spin()

