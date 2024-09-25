import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from tf_utils import TransformationUtils  # Import Part 3 functions

class DepthHandler:
    def __init__(self):
        self.c = None  # Coordinates from the /co_ords topic
        self.depth = None
        self.bridge = CvBridge()
        
        # Camera parameters
        self.cx = None
        self.cy = None
        self.fx = None
        self.fy = None
        
        # List to store valid coordinates
        self.valid_coords_count = 0
        self.coordinates_list = []
        
        # Transformation utilities (Part 3)
        self.transformation_utils = TransformationUtils()

    def camera_info_callback(self, data):
        """Callback to set intrinsic camera parameters."""
        self.fx = 360.01333
        self.fy = 360.013366699
        self.cx = 243.87228
        self.cy = 137.9218444

    def callback_coords(self, data):
        """Callback to handle incoming coordinates."""
        self.c = data.data

    def callback_depth(self, data):
        """Callback to handle depth image data and perform 3D transformations."""
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

            # Use transformation utils from Part 3
            robot_coords = self.transformation_utils.convert_to_robot_frame(camera_coords)

            X_robot, Y_robot, Z_robot = robot_coords[:3]

            # Validate and store coordinates
            if not np.isnan(X_robot) and not np.isnan(Y_robot) and not np.isnan(Z_robot):
                self.coordinates_list.append([X_robot, Y_robot, Z_robot])
                self.valid_coords_count += 1

                rospy.loginfo(f"3D coordinates in robot frame: X={X_robot}, Y={Y_robot}, Z={Z_robot}")

                if self.valid_coords_count >= 10:
                    self.save_avg_coordinates()

    def save_avg_coordinates(self):
        """Function to save the average of 10 valid coordinates."""
        avg_coords = np.mean(self.coordinates_list, axis=0)
        file_path = "/home/architsharma/catkin_workspace/.../inverse_kinematics.txt"
        
        with open(file_path, "w") as f:
            f.write(f"{avg_coords}")

        rospy.loginfo(f"Average 3D coordinates: X={avg_coords[0]}, Y={avg_coords[1]}, Z={avg_coords[2]}")
        rospy.signal_shutdown("Collected 10 valid coordinates.")
