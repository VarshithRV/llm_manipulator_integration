import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest

class RosServiceWrapper:
    def __init__(self):
        rospy.init_node('ros_service_wrapper')
        self.service = rospy.Service('/gripper', SetBool, self.callback)
        self.setpoint_pub = rospy.Publisher('/gripper', Bool, queue_size=1)
        self.setpoint_pub_timer = rospy.Timer(rospy.Duration(0.05), self.setpoint_pub_callback)
        self.setpoint = False

    def callback(self, req):
        self.setpoint = req.data
        return SetBoolResponse(success=True)
    
    def setpoint_pub_callback(self, event):
        self.setpoint_pub.publish(Bool(data=self.setpoint))

if __name__ == "__main__":
    RosServiceWrapper()
    rospy.loginfo("ROS Service Wrapper for gripper started")
    rospy.spin()