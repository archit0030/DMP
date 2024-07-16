#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import moveit_commander
import os
import pandas as pd
from math import pi, degrees
from geometry_msgs.msg import PoseStamped


def joint_state_callback(data):
    joint_angles = data.position
    joint_velocities = data.velocity
    joint_effort = data.effort
    print("Joint angles:", joint_angles)
    print("Joint velocities:", joint_velocities)
    print("Joint effort:", joint_effort)
    dataa = {}
    for i in range(len(joint_angles)):
        dataa[f'joint{i}'] = joint_angles[i]
    dataFrame.loc[dataFrame.size] = dataa
    for i in range(len(joint_velocities)):
        dataa[f'joint{i}'] = joint_velocities[i]
    dataFrame.loc[dataFrame.size] = dataa
    for i in range(len(joint_effort)):
        dataa[f'joint{i}'] = joint_effort[i]
    dataFrame.loc[dataFrame.size] = dataa
    
def manipulator_subscriber():
    moveit_commander.roscpp_initialize("my_gen3")
    rospy.init_node('joint_state_subscriber')
    arm_group_name = "arm"
    rospy.Subscriber('/my_gen3/joint_states', JointState, joint_state_callback)
    rospy.spin()
    print(f"Saved to {dataFileLoaction}")
    dataFrame.to_csv(dataFileLoaction)
       
dataFileLoaction = os.path.join("~/catkin_workspace", "Joint_State_data.csv")
dataFrame = pd.DataFrame(columns=['joint' + str(j) for j in range(0,7)])
try:
    dataFrame = pd.read_csv(dataFileLoaction)
    print("Loaded Data")
except:
    print("new file created")
    pass


if __name__ == '__main__':
    manipulator_subscriber()
    