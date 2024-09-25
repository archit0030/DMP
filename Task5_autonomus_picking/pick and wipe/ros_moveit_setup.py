#!/usr/bin/env python3

import rospy
import moveit_commander
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from depth_handling import DepthHandler

class ROSMoveItSetup:
    def __init__(self):
        rospy.init_node('depth_to_3d_converter', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.link_name = 'base_link'

        # Initialize DepthHandler, which handles the depth image and coordinate data
        self.depth_handler = DepthHandler()

        # Set up subscribers
        rospy.Subscriber('/camera/depth/image_rect', Image, self.depth_handler.callback_depth)
        rospy.Subscriber('/co_ords', Float32MultiArray, self.depth_handler.callback_coords)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_handler.camera_info_callback)

        rospy.spin()


if __name__ == '__main__':
    try:
        ROSMoveItSetup()
    except rospy.ROSInterruptException:
        pass
