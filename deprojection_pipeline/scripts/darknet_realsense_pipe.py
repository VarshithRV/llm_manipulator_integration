#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

def callback(data):
    # Republish the received image message to /camera/rgb/image_raw
    pub.publish(data)

def image_republisher():
    global pub

    # Initialize the ROS node
    rospy.init_node('image_republisher', anonymous=True)

    # Create a publisher for the /camera/rgb/image_raw topic
    pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=10)

    # Subscribe to the /camera/color/image_raw topic
    rospy.Subscriber('/camera/color/image_raw', Image, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        image_republisher()
    except rospy.ROSInterruptException:
        pass
