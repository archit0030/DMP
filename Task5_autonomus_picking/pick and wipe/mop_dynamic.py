#!/usr/bin/env python3


import sys
import rospy
import time
import math

import actionlib
import os

from kortex_driver.srv import *
from std_msgs.msg import String, Float32
from kortex_driver.msg import *

class ExampleWaypointActionClient:
    def __init__(self):
        try:
            rospy.init_node('my_code')

            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "my_gen3")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            # Init the services
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def FillCartesianWaypoint(self, new_x, new_y, new_z, new_theta_x, new_theta_y, new_theta_z, blending_radius):
        cartesianWaypoint = CartesianWaypoint()

        cartesianWaypoint.pose.x = new_x
        cartesianWaypoint.pose.y = new_y
        cartesianWaypoint.pose.z = new_z
        cartesianWaypoint.pose.theta_x = new_theta_x
        cartesianWaypoint.pose.theta_y = new_theta_y
        cartesianWaypoint.pose.theta_z = new_theta_z
        cartesianWaypoint.reference_frame = CartesianReferenceFrame.CARTESIAN_REFERENCE_FRAME_BASE
        cartesianWaypoint.blending_radius = blending_radius
       
        return cartesianWaypoint

    def example_subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True

    def example_clear_faults(self):
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)
            return True

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                return self.wait_for_action_end_or_abort()


    def example_send_gripper_command(self, value):
        # Initialize the request
        # Close the gripper
        req = SendGripperCommandRequest()
        finger = Finger()
        finger.finger_identifier = 0
        finger.value = value
        req.input.gripper.finger.append(finger)
        req.input.mode = GripperMode.GRIPPER_POSITION

        rospy.loginfo("Sending the gripper command...")

        # Call the service 
        try:
            self.send_gripper_command(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call SendGripperCommand")
            return False
        else:
            time.sleep(0.1)
            return True

    def home_position(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        # Get the current Cartesian pose
        rospy.wait_for_service('/' + self.robot_name + '/base/get_measured_cartesian_pose')
        try:
            get_pose_service = rospy.ServiceProxy('/' + self.robot_name + '/base/get_measured_cartesian_pose', GetMeasuredCartesianPose)
            current_pose = get_pose_service().output
            print(current_pose)
            x_init = current_pose.x
            y_init = current_pose.y
            z_init = current_pose.z
            theta_x_init = current_pose.theta_x
            theta_y_init = current_pose.theta_y
            theta_z_init = current_pose.theta_z
        except rospy.ServiceException:
            rospy.logerr("Failed to get current pose")
            return False

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
            print("ok")
        else:
            goal.trajectory.append(self.FillCartesianWaypoint(-0.072, -0.006, 0.611, math.radians(111.8), math.radians(0.2), math.radians(87.7), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.434, -0.014, 0.453, math.radians(99), math.radians(-0.6), math.radians(87.7), 0))
        # Call the service
        rospy.loginfo("Sending goal (Cartesian waypoint) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            client.wait_for_result()
            return True

    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()
        
        # Get the current Cartesian pose
        rospy.wait_for_service('/' + self.robot_name + '/base/get_measured_cartesian_pose')
        try:
            get_pose_service = rospy.ServiceProxy('/' + self.robot_name + '/base/get_measured_cartesian_pose', GetMeasuredCartesianPose)
            current_pose = get_pose_service().output
            print(current_pose)
            x_init = current_pose.x
            y_init = current_pose.y
            
            theta_x_init = current_pose.theta_x
            theta_y_init = current_pose.theta_y
            theta_z_init = current_pose.theta_z
        except rospy.ServiceException:
            rospy.logerr("Failed to get current pose")
            return False

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            print("ok")
        else:
            
            directory = "/home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics/dynamic_moping/"
            file_path = os.path.join(directory, "inverse_kinematics.txt")
            
            # Ensure the directory exists
            os.makedirs(directory, exist_ok=True)
            
            with open(file_path, "r") as f:
                coords = f.readline().strip().replace('[', '').replace(']', '').split()
                coords = [float(coord) for coord in coords]

            
            if len(coords) != 3:
                rospy.logerr("Invalid coordinates in the file")
                return False

            x, y, z = coords
            goal.trajectory.append(self.FillCartesianWaypoint(x+0.055,  y,  0.434, math.radians(147.4), math.radians(-1.1), math.radians(89.1), 0))
            
            

            
        # time.sleep(2)  # import time
        # Call the service
        rospy.loginfo("Sending goal(Cartesian waypoint) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            client.wait_for_result()
            return True
        
        



        
#----------------------------""""going to place the object""""----------------------------#
    def going_mop(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        # Get the current Cartesian pose
        rospy.wait_for_service('/' + self.robot_name + '/base/get_measured_cartesian_pose')
        try:
            get_pose_service = rospy.ServiceProxy('/' + self.robot_name + '/base/get_measured_cartesian_pose', GetMeasuredCartesianPose)
            current_pose = get_pose_service().output
            print(current_pose)
            x_init = current_pose.x
            y_init = current_pose.y
            z_init = current_pose.z
            theta_x_init = current_pose.theta_x
            theta_y_init = current_pose.theta_y
            theta_z_init = current_pose.theta_z
        except rospy.ServiceException:
            rospy.logerr("Failed to get current pose")
            return False

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
            print("ok")
        else:

            goal.trajectory.append(self.FillCartesianWaypoint(x_init, y_init, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(x_init, 0.248, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(x_init, -0.369, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(x_init, 0.248, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(x_init+0.055, -0.369, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(x_init+0.055, 0.248, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(x_init, y_init, z_init , math.radians(theta_x_init), math.radians(theta_y_init), math.radians(theta_z_init), 0))
        # Call the service
        rospy.loginfo("Sending goal (Cartesian waypoint) to action server...")
        try:
            client.send_goal(goal)
        except rospy.ServiceException:
            rospy.logerr("Failed to send goal.")
            return False
        else:
            client.wait_for_result()
            return True
        
        
    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/waypoint_action_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Activate the action notifications
            # success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************


            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.0)
            else:
                rospy.logwarn("No gripper is present on the arm.")
                
                
            #*******************************************************************************
            #*******************************************************************************
            # Move the robot to the Home position with an Action
            success &= self.home_position()
            # success &= self.example_clear_faults()
            #*******************************************************************************
            
            
            #*******************************************************************************
            #*******************************************************************************
            # Move the robot near or parallel to the object
            os.system('python3 /home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics/dynamic_moping/reach_near_to_object.py')
            # success &= self.example_clear_faults()
            #*******************************************************************************
            
            
            
            #*******************************************************************************
            #*******************************************************************************
            # Move the robot to pick the object
            success &= self.example_cartesian_waypoint_action()
            
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.86)
            else:
                rospy.logwarn("No gripper is present on the arm.")  
            #*******************************************************************************
            #*******************************************************************************
            
            # success &= self.taking_height()
            
            # success &= self.going_right_to_goal()
            # success &= self.going_down()
            # success &= self.example_send_gripper_command(0.0)
            # success &= self.example_clear_faults()
            #*******************************************************************************
            #*******************************************************************************
            success &= self.going_mop()
            success &= self.example_send_gripper_command(0.0)
            success &= self.home_position()

            print("--------------------------------------------success----------------------------------------------------------")


        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = ExampleWaypointActionClient()
    ex.main()
