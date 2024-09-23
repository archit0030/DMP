#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import sys
import rospy 
import rospkg
import pandas as pd
import os
import moveit_commander
from geometry_msgs.msg import PoseStamped
from math import pi

class ExampleMoveItTrajectories(object):
    """ExampleMoveItTrajectories1"""

    def __init__(self):
        # Initialize the node
        super(ExampleMoveItTrajectories, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('my_code')

        try:
            self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            arm_group_name = "arm"
            self.robot = moveit_commander.RobotCommander("robot_description")
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
            rate = rospy.Rate(0.001)
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True



    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
            return val
        except:
            return False 


    def reach_named_position(self, target):
        arm_group = self.arm_group
        
        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)

        (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
        return arm_group.execute(trajectory_message, wait=True)

    def reach_joint_angles(self, tolerance):
        arm_group = self.arm_group
        success = True

            # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.030087605450361217
            joint_positions[1] = -0.6060507755793703
            joint_positions[2] = -3.1407883789405893
            joint_positions[3] = -1.4752032247309383
            joint_positions[4] = -0.15911908144227294
            joint_positions[5] = -1.6610236918945631
            joint_positions[6] = 1.704797937574141
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = -0.05205947298886926
            joint_positions[1] = 0.20967884519753358
            joint_positions[2] = 3.1181653581125843
            joint_positions[3] = -0.9702764041674037
            joint_positions[4] = -0.08024210890852412
            joint_positions[5] = -1.729202746329464
            joint_positions[6] =  1.6488212209378745
        arm_group.set_joint_value_target(joint_positions)
        success &= arm_group.go(wait=True)
        
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.0741444770200292
            joint_positions[1] = 0.21864820530135232
            joint_positions[2] = 2.886718549889286
            joint_positions[3] = -1.4713901106821634
            joint_positions[4] =  0.008530907560891376
            joint_positions[5] =  -1.3130406827949237
            joint_positions[6] = 1.5710891413567507
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success


    def reach_joint_angles1(self, tolerance):
        arm_group = self.arm_group
        success = True

            # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.24271808826008387
            joint_positions[1] = 0.20602828384379385
            joint_positions[2] = 2.9607192744671
            joint_positions[3] = -1.4671684677221695
            joint_positions[4] = -0.04060202134713009
            joint_positions[5] = -1.3381436392300108
            joint_positions[6] = 1.8183087874521524
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.4448524976950456
            joint_positions[1] = 0.2693289764386441
            joint_positions[2] = 3.0392841245210915
            joint_positions[3] = -1.3824373344802083
            joint_positions[4] = -0.08138567028060262
            joint_positions[5] =  -1.387941023188966
            joint_positions[6] = 2.100219702013308
        arm_group.set_joint_value_target(joint_positions)
        success &= arm_group.go(wait=True)
        
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.5829097699193445
            joint_positions[1] = 0.3632585349276843
            joint_positions[2] = 3.082287784539255
            joint_positions[3] = -1.2488611636851035
            joint_positions[4] = -0.10249708087387965
            joint_positions[5] = -1.450019308198713
            joint_positions[6] = 2.276335077532466
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.738968146899816
            joint_positions[1] = 0.5413787048259403
            joint_positions[2] = 3.1158894206450025
            joint_positions[3] = -0.9664377237721631
            joint_positions[4] = -0.12209475070355502
            joint_positions[5] = -1.5794500024409999
            joint_positions[6] = 2.447823210657611
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.8137614282180375
            joint_positions[1] = 0.678642954256641
            joint_positions[2] = 3.126821430603669
            joint_positions[3] = -0.7233225415487912
            joint_positions[4] = -0.13196282780673485
            joint_positions[5] = -1.6963164352925109
            joint_positions[6] = 2.51365921596546
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.738968146899816
            joint_positions[1] = 0.5413787048259403
            joint_positions[2] = 3.1158894206450025
            joint_positions[3] = -0.9664377237721631
            joint_positions[4] = -0.12209475070355502
            joint_positions[5] = -1.5794500024409999
            joint_positions[6] = 2.447823210657611
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.5829097699193445
            joint_positions[1] = 0.3632585349276843
            joint_positions[2] = 3.082287784539255
            joint_positions[3] = -1.2488611636851035
            joint_positions[4] = -0.10249708087387965
            joint_positions[5] = -1.450019308198713
            joint_positions[6] = 2.276335077532466
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.4448524976950456
            joint_positions[1] = 0.2693289764386441
            joint_positions[2] = 3.0392841245210915
            joint_positions[3] = -1.3824373344802083
            joint_positions[4] = -0.08138567028060262
            joint_positions[5] =  -1.387941023188966
            joint_positions[6] = 2.100219702013308
        arm_group.set_joint_value_target(joint_positions)
        success &= arm_group.go(wait=True)
        
        
        
        self.arm_group.set_goal_joint_tolerance(tolerance)
        

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.24271808826008387
            joint_positions[1] = 0.20602828384379385
            joint_positions[2] = 2.9607192744671
            joint_positions[3] = -1.4671684677221695
            joint_positions[4] = -0.04060202134713009
            joint_positions[5] = -1.3381436392300108
            joint_positions[6] = 1.8183087874521524
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.0741444770200292
            joint_positions[1] = 0.21864820530135232
            joint_positions[2] = 2.886718549889286
            joint_positions[3] = -1.4713901106821634
            joint_positions[4] =  0.008530907560891376
            joint_positions[5] =  -1.3130406827949237
            joint_positions[6] = 1.5710891413567507
        arm_group.set_joint_value_target(joint_positions)
        
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)

        # Show joint positions after movement
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success
    
    
    
    
    def reach_joint_angles2(self, tolerance):
        arm_group = self.arm_group
        success = True

            # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")
        for p in joint_positions: rospy.loginfo(p)

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        if self.degrees_of_freedom == 7:
            joint_positions[0] = 0.030087605450361217
            joint_positions[1] = -0.6060507755793703
            joint_positions[2] = -3.1407883789405893
            joint_positions[3] = -1.4752032247309383
            joint_positions[4] = -0.15911908144227294
            joint_positions[5] = -1.6610236918945631
            joint_positions[6] = 1.704797937574141
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        success &= arm_group.go(wait=True)
        new_joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions after movement :")
        for p in new_joint_positions: rospy.loginfo(p)
        return success
def main():
    example = ExampleMoveItTrajectories()

    # For testing purposes
    success = example.is_init_success
    try:
        rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
    except:
        pass
    
    
    
    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0)
        print (success)


    if success:
        rospy.loginfo("Reaching Joint Angles...")
        success &= example.reach_joint_angles(tolerance=0.01)  
        print(success)
        
    if example.is_gripper_present and success:
        # rospy.loginfo("Opening the gripper...")
        # success &= example.reach_gripper_position(0)
        # print (success)

        rospy.loginfo("Closing the gripper 95%...")
        success &= example.reach_gripper_position(0.70)
        print (success)
        
        
    if success:
        rospy.loginfo("Reaching Joint Angles...")
        success &= example.reach_joint_angles1(tolerance=0.01)  
        print(success)
        
    
        
        
    if example.is_gripper_present and success:
        rospy.loginfo("Opening the gripper...")
        success &= example.reach_gripper_position(0.1)
        print (success)
        
    if success:
        rospy.loginfo("Reaching Joint Angles...")
        success &= example.reach_joint_angles2(tolerance=0.01)  
        print(success)
    
    

    # if success:
    #     rospy.loginfo("Reaching Named Target Home...")
    #     success &= example.reach_named_position("home")
    #     print(success)

    # For testing purposes
    rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

    if not success:
        rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
    main()