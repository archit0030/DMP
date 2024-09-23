#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2021 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import rospy
import time
import math

import actionlib


from kortex_driver.srv import *
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


    def example_cartesian_waypoint_action(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            goal.trajectory.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
        else:
            goal.trajectory.append(self.FillCartesianWaypoint(0.167, 0.043,  0.604, math.radians(144.5), math.radians(-3.4), math.radians(100.9), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.235, 0.404,  0.533,  math.radians(158), math.radians(-3.3), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.239, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.227, -0.553,  0.372,  math.radians(158.3), math.radians(-4.1), math.radians(100.1), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.239, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.334, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.341, -0.477,  0.372, math.radians(158.4), math.radians(-3.7), math.radians(100.1), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.334, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.412,  0.485,   0.371, math.radians(158.7), math.radians(-3.2), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.481,  -0.411,  0.37, math.radians(158.9), math.radians(-3.4), math.radians(100.1), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.412,  0.485,   0.371, math.radians(158.7), math.radians(-3.2), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.239, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.167, 0.043,  0.604, math.radians(144.5), math.radians(-3.4), math.radians(100.9), 0))
            
        time.sleep(2)  # import time
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
        
        
        
        
        
    def example_cartesian_waypoint_action1(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            goal.trajectory.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
        else:
            # goal.trajectory.append(self.FillCartesianWaypoint(0.167, 0.043,  0.604, math.radians(144.5), math.radians(-3.4), math.radians(100.9), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.239, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.227, -0.553,  0.372,  math.radians(158.3), math.radians(-4.1), math.radians(100.1), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.239, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.334, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.341, -0.477,  0.372, math.radians(158.4), math.radians(-3.7), math.radians(100.1), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.334, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.412,  0.485,   0.371, math.radians(158.7), math.radians(-3.2), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.481,  -0.411,  0.37, math.radians(158.9), math.radians(-3.4), math.radians(100.1), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.412,  0.485,   0.371, math.radians(158.7), math.radians(-3.2), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.239, 0.485,  0.371,  math.radians(158.7), math.radians(-3.3), math.radians(100.2), 0))
            # goal.trajectory.append(self.FillCartesianWaypoint(0.167, 0.043,  0.604, math.radians(144.5), math.radians(-3.4), math.radians(100.9), 0))
            

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
        
        
        
    def example_cartesian_waypoint_action2(self):
        self.last_action_notif_type = None

        client = actionlib.SimpleActionClient('/' + self.robot_name + '/cartesian_trajectory_controller/follow_cartesian_trajectory', kortex_driver.msg.FollowCartesianTrajectoryAction)

        client.wait_for_server()

        goal = FollowCartesianTrajectoryGoal()

        config = self.get_product_configuration()

        if config.output.model == ModelId.MODEL_ID_L31:
        
            goal.trajectory.append(self.FillCartesianWaypoint(0.439,  0.194,  0.448, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.200,  0.150,  0.400, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.350,  0.050,  0.300, math.radians(90.6), math.radians(-1.0), math.radians(150), 0))
        else:
            goal.trajectory.append(self.FillCartesianWaypoint(0.235, 0.404,  0.533,  math.radians(158), math.radians(-3.3), math.radians(100.2), 0))
            goal.trajectory.append(self.FillCartesianWaypoint(0.167, 0.043,  0.604, math.radians(144.5), math.radians(-3.4), math.radians(100.9), 0))
            

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
    

    def main(self):
        # For testing purposes
        success = self.is_init_success
        try:
            rospy.delete_param("/kortex_examples_test_results/waypoint_action_python")
        except:
            pass

        if success:
            #*******************************************************************************
            # Make sure to clear the robot's faults else it won't move if it's already in fault
            success &= self.example_clear_faults()
            #*******************************************************************************
            
            #*******************************************************************************
            # Activate the action notifications
            success &= self.example_subscribe_to_a_robot_notification()
            #*******************************************************************************

            #*******************************************************************************
            # Move the robot to the Home position with an Action
            # success &= self.example_home_the_robot()
            #*******************************************************************************
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0)
            else:
                rospy.logwarn("No gripper is present on the arm.")
            #*******************************************************************************
            # Example of Cartesian waypoint using an action client
            success &= self.example_cartesian_waypoint_action()
            
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0.87)
            else:
                rospy.logwarn("No gripper is present on the arm.")  
            
            success &= self.example_cartesian_waypoint_action1()
            
            if self.is_gripper_present:
                success &= self.example_send_gripper_command(0)
            else:
                rospy.logwarn("No gripper is present on the arm.")
                
            success &= self.example_cartesian_waypoint_action2()
            #*******************************************************************************
            #*******************************************************************************
            # Move back the robot to the Home position with an Action
            # success &= self.example_home_the_robot()
            #*******************************************************************************

        # For testing purposes
        rospy.set_param("/kortex_examples_test_results/waypoint_action_python", success)

        if not success:
            rospy.logerr("The example encountered an error.")


if __name__ == "__main__":
    ex = ExampleWaypointActionClient()
    ex.main()
