#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import os
import pandas as pd
from math import pi, degrees
from geometry_msgs.msg import PoseStamped

global dataFrame

def joint_state_callback(data):
    global dataFrame

    joint_angles = data.position
    joint_velocities = data.velocity
    joint_effort = data.effort
    print("Joint angles:", joint_angles)
    print("Joint velocities:", joint_velocities)
    print("Joint effort:", joint_effort)
    dataa = {}
    for i in range(len(joint_angles)):
        dataa[f'joint_{i}'] = float(joint_angles[i])
    for i in range(len(joint_velocities)):
        dataa[f'vel_{i}'] = float(joint_velocities[i])
    for i in range(len(joint_effort)):
        dataa[f'eff_{i}'] = float(joint_effort[i])
    dataFrame = pd.concat([dataFrame, pd.DataFrame([dataa])])
    
def manipulator_subscriber():
    rospy.init_node('joint_state_subscriber')
    rospy.Subscriber('/my_gen3/joint_states', JointState, joint_state_callback)
    rospy.spin()
    print(f"Saved to {dataFileLoaction}")
    print(dataFrame.head(5))
    dataFrame.to_csv(dataFileLoaction)
       


if __name__ == '__main__':
	dataFileLoaction = os.path.join("/home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics", "my_csv.csv")
	dataFrame = pd.DataFrame(columns=[k + str(j) for j in range(0,13) for k in ["joint_", "vel_", "eff_"]])
	# print(dataFrame.head())
	try:
	    dataFrame = pd.read_csv(dataFileLoaction)
	    print("Loaded Data")
	except:
	    print("new file created")
    	   
	manipulator_subscriber()
