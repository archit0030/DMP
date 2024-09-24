#!/usr/bin/env python3

#steps to execute - 
# save this file in a folder kortex_examples/src/moveit/
# make python file executable - chmod +x joint_space.py
# change in the launch file located in "kortex_examples/launch/" "change the path to the file - joint_space.py"
# roslaunch kortex_driver kortex_driver.launch
# rostopic echo /my_gen3/joint_states
# roslaunch kortex_examples cartesian.launch

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

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
    except Exception as e:
      print (e)
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
      joint_positions[0] = 0.11675637771946366
      joint_positions[1] = -1.0583961435803806
      joint_positions[2] = -3.061668526115425
      joint_positions[3] = -2.1198932719340497
      joint_positions[4] = -0.23126944169469965
      joint_positions[5] = -0.5324809504659314
      joint_positions[6] = 1.8588648692704786
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)
    
    
    
    
    #2222
    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = 0.22183644188985435
      joint_positions[1] = 0.4321109030651842
      joint_positions[2] = 2.56041127521791
      joint_positions[3] = -1.6471715258006245
      joint_positions[4] = -0.8679364497978383
      joint_positions[5] = 0.4944966500227808
      joint_positions[6] = 2.562321294351715
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)
    
    
    
    
    
    

    # Show joint positions after movement
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

  success &= example.reach_gripper_position(0)
  
  
  if success:
    rospy.loginfo("Reaching Joint Angles...")  
    success &= example.reach_joint_angles(tolerance=0.01) #rad
    print (success)
  
#   if success:
#     rospy.loginfo("Reaching Named Target Home...")
#     success &= example.reach_named_position("home")
#     print (success)

#   if success:
#     rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     actual_pose.position.z -= 0.2
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
#     print (success)
    
#   if example.degrees_of_freedom == 7 and success:
#     rospy.loginfo("Reach Cartesian Pose with constraints...")
#     # Get actual pose
#     actual_pose = example.get_cartesian_pose()
#     actual_pose.position.y -= 0.3
    
#     # Orientation constraint (we want the end effector to stay the same orientation)
#     constraints = moveit_msgs.msg.Constraints()
#     orientation_constraint = moveit_msgs.msg.OrientationConstraint()
#     orientation_constraint.orientation = actual_pose.orientation
#     constraints.orientation_constraints.append(orientation_constraint)

#     # Send the goal
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=constraints)

  if example.is_gripper_present and success:
    # rospy.loginfo("Opening the gripper...")
    # success &= example.reach_gripper_position(0)
    # print (success)

    rospy.loginfo("Closing the gripper 50%...")
    success &= example.reach_gripper_position(0.6)
    print (success)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()
  
  
  
#steps for executing the code - roslaunch kortex_driver kortex_driver.launch