import rospy
import numpy as np
import tf2_ros
import tf

def get_transform(self, target_frame, source_frame):
    try:
        transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))
        return transform
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn(f"Failed to get transform from {source_frame} to {target_frame}")
        return None

def transform_to_matrix(self, transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    print(translation.y,translation.x)
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


def transform_coordinates(self, object_coordinates_camera_frame, transformation_matrix):
    # Add a homogeneous coordinate (1) to the object coordinates
    object_coordinates_camera_frame_homogeneous = np.append(object_coordinates_camera_frame, 1)

    # Perform the coordinate transformation
    object_coordinates_end_effector_frame_homogeneous = np.dot(transformation_matrix, object_coordinates_camera_frame_homogeneous)

    # Remove the homogeneous coordinate (1) from the result
    object_coordinates_end_effector_frame = object_coordinates_end_effector_frame_homogeneous[:-1]

    return object_coordinates_end_effector_frame