import rospy
import numpy as np
import tf2_ros
import tf

class TransformationUtils:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def convert_to_robot_frame(self, camera_coords):
        """Converts 3D coordinates from the camera frame to the robot frame."""
        transform = self.get_transform("base_link", "camera_link")
        if transform:
            transformation_matrix = self.transform_to_matrix(transform)
            return self.transform_coordinates(camera_coords, transformation_matrix)
        else:
            rospy.logwarn("Transform not available")
            return np.full(3, np.nan)

    def get_transform(self, target_frame, source_frame):
        """Get transformation from one frame to another."""
        try:
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed to get transform from {source_frame} to {target_frame}")
            return None

    def transform_to_matrix(self, transform):
        """Convert transform to a 4x4 transformation matrix."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        translation_matrix = np.array([
            [1, 0, 0, translation.x],
            [0, 1, 0, translation.y],
            [0, 0, 1, translation.z],
            [0, 0, 0, 1]
        ])

        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        rotation_matrix = tf.transformations.quaternion_matrix(quaternion)
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)
        return transformation_matrix

    def transform_coordinates(self, camera_coords, transformation_matrix):
        """Apply transformation to the coordinates."""
        camera_coords_homogeneous = np.append(camera_coords, 1)
        robot_coords_homogeneous = np.dot(transformation_matrix, camera_coords_homogeneous)
        return robot_coords_homogeneous[:-1]
