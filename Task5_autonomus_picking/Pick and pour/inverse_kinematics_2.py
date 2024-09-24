#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
import tf
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import moveit_commander
import sys
# sys.path.append('/home/surya/catkin_ws/src/Task5_autonomus_picking/scripts') # add path if needed
from get_transform2 import *

class DepthTo3DConverter:
    def __init__(self):
        self.c = None
        self.depth = None
        self.bridge = CvBridge()
        
        
        self.valid_coords_count = 0
        self.coordinates_list = []
        
        
        self.cx = None
        self.cy = None
        self.fx = None
        self.fy = None

        rospy.init_node('my_code', anonymous=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.robot = moveit_commander.RobotCommander()
        self.link_name = 'base_link'  

        rospy.Subscriber('/camera/depth/image_rect', Image, self.callback_depth)
        rospy.Subscriber('/co_ords2', Float32MultiArray, self.callback_coords)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        
        rospy.spin()

    def camera_info_callback(self, data):

        
        self.fx =360.01333 #335.234436#
        self.fy = 360.013366699#335.234436#
        self.cx = 243.87228#242.452332 #
        self.cy = 137.9218444#142.120667#
        
        


    def callback_coords(self, data):
        self.c = data.data

    def callback_depth(self, data):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            rospy.logwarn("Waiting for camera intrinsic parameters")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        if self.c is not None:
            u = int((self.c[0] + 0.055) * 480)
            v = int((self.c[1] + 0.12) * 270)

            depth_value = frame[v, u]

            X_camera = ((u - self.cx) * depth_value) / self.fx
            Y_camera = ((v - self.cy) * depth_value) / self.fy

            Z_camera = depth_value 

            camera_coords = np.array([X_camera, Y_camera, Z_camera])
            transform = self.get_transform("base_link", "camera_link")
            
            max_prints = 10
            print_count = 0
            
            if transform:
                transformation_matrix = self.transform_to_matrix(transform)
                robot_coords = self.transform_coordinates(camera_coords, transformation_matrix)

                X_robot, Y_robot, Z_robot = robot_coords[:3]

                        
                if not np.isnan(X_robot) and not np.isnan(Y_robot) and not np.isnan(Z_robot):
                    self.coordinates_list.append([X_robot, Y_robot, Z_robot])
                    self.valid_coords_count += 1
                    rospy.loginfo(f"3D coordinates in robot frame: X={X_robot}, Y={Y_robot}, Z={Z_robot}")

                    if self.valid_coords_count >= 10:
                        import os
                        avg_coords = np.mean(self.coordinates_list, axis=0)
                        directory = "/home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics/dynamic_pouring/"
                        file_path = os.path.join(directory, "inverse_kinematics_2.txt")
                        
                        # Ensure the directory exists
                        os.makedirs(directory, exist_ok=True)
                        
                        with open(file_path, "w") as f:
                            f.write(f"{avg_coords}")
        
                        rospy.loginfo(f"Average 3D coordinates in robot frame: X={avg_coords[0]}, Y={avg_coords[1]}, Z={avg_coords[2]}")
                        rospy.signal_shutdown("Collected 10 valid coordinates and calculated the average.")


                # rospy.loginfo(f"3D coordinates in robot frame: X={X_robot}, Y={Y_robot}, Z={Z_robot}")
            else:
                rospy.logwarn("Transform not available")
        else:
            rospy.logwarn("Waiting for coordinates")

if __name__ == '__main__':
    try:
        DepthTo3DConverter()
    except rospy.ROSInterruptException:
        pass
