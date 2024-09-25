#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from yolo_processing import process_frame
import cv2


# Global variables
bridge = CvBridge()
PPI = 57  # Pixels per inch for distance conversion

# Callback function for ROS Subscriber
def callback(data):
    global bridge

    co_ords = Float32MultiArray()
    co_ords.data = (0.0, 0.0, 0.0, 0.0)
    distance = Float32MultiArray()
    distance.data = (0.0, 0.0)

    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    co_ords, distance, processed_frame = process_frame(frame, PPI)

    # Publish the data
    pub.publish(distance)
    pub1.publish(co_ords)

    # Show the processed frame with bounding boxes and distances
    cv2.imshow("Image", processed_frame)
    cv2.waitKey(1)

# Main function to initialize ROS node
def main():
    global pub, pub1

    rospy.init_node('ros_yolo_node', anonymous=True)

    # Initialize the ROS subscribers and publishers
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, callback)
    pub = rospy.Publisher('/std_msgs/Float32MultiArray', Float32MultiArray, queue_size=10)
    pub1 = rospy.Publisher('co_ords', Float32MultiArray, queue_size=10)

    # Keep the node running until manually stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # Clean up any open windows
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
