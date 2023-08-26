#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from vision_msgs.msg import Detection2DArray

def image_publisher():
    rospy.init_node("image_publisher_node")
    # pub = rospy.Publisher("/bottom_camera/image_raw", Image, queue_size=10)
    pub = rospy.Publisher("/front_camera/image_raw", Image, queue_size=10)
    rate = rospy.Rate(0.5)  # Publish at 1 Hz

    bridge = CvBridge()
    image_path = "/home/orca-auv/catkin_ws/src/yolov8_ros/25.png"

    image = cv2.imread(image_path)
    if image is None:
        print("Failed to load image")
        return

    while not rospy.is_shutdown():
        img_msg = bridge.cv2_to_imgmsg(image, "bgr8")  # Convert OpenCV image to ROS Image message
        pub.publish(img_msg)
        # rate.sleep()
        # TODO: rate.sleep() doesn't work

if __name__ == "__main__":
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass
