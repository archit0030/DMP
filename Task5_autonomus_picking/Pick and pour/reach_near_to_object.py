#!/usr/bin/env python3

import rospy
from kortex_driver.msg import TwistCommand
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 , Empty, Float32MultiArray
import subprocess
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

kp = 0.8
def callback(data):
    global depth
    # Extract depth value from callback data
    
    msgg = data.data
    msg= msgg[0]*kp , msgg[1]*kp
    # print(msg)
    # rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)


    my = TwistCommand()
    my.reference_frame  = 0
    zx = Twist()
    my.twist.linear_y = 0.01
    my.duration = 1

    if msg[0] <= 29 and msg[0] >= 8:
        my.twist.linear_y = 0.1 
    elif msg[0] >= -29 and msg[0] <= -8:
        my.twist.linear_y = -0.1
    elif msg[0] <8 and msg[0] >= 3:
        my.twist.linear_y = 0.01
        
    elif msg[0] > -8 and msg[0]<= -3:
        my.twist.linear_y = -0.01
    elif msg[0] <3 and msg[0] >= 0.2:
        my.twist.linear_y = 0
    elif msg[0] > -3 and msg[0]<= -0.2:
        my.twist.linear_y = 0
      
      
      
      
    #for z axis 
    if msg[1]<= 29 and msg[1]>= 8:
        my.twist.linear_z = 0.05     
    elif msg[1]>= -29 and msg[1]<= -8:
        my.twist.linear_z = -0.05
    elif msg[1]<8 and msg[1]>= 3:
        my.twist.linear_z = 0.009  
    elif msg[1]> -8 and msg[1]<= -3:
        my.twist.linear_z = -0.009
    elif msg[1]<3 and msg[1]>= 0.2:
        my.twist.linear_z = 0
    elif msg[1]> -3 and msg[1]<= -0.2:
        my.twist.linear_z = 0

    
    
    
    #for x axis
    if depth <= 1.5 and depth >= 0.5:
        my.twist.linear_x = 0.05
        
    elif depth <= 0.5 and depth >= 0.4:
        my.twist.linear_x = 0.02
        
    
    
          
    else:
        print("Value out of range")
    pub.publish(my)
    if my.twist.linear_y == 0:
        if my.twist.linear_z == 0:
            if my.twist.linear_x == 0:
                rospy.loginfo("Stopping robot")
                # depth = 0.27
                os.system('python3 /home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics/dynamic_pouring/inverse_kinematics.py')
                rospy.signal_shutdown("Reached target velocity")
            # rate.sleep()  


def callback_depth(data):
    global c , depth

    pub_x = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=1)

    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(data, "passthrough")


    except CvBridgeError as e:
        print(e)


    distance = frame[int((c[1]+0.12)*270), int((c[0]+0.055)*480)]
    depth = (distance - 0.09)
    print('Depth at center: ', depth, 'cm')
    
    
    
    
def callback1(data):

  global c

  c = data.data
  
  
  
def main():
    rospy.init_node('my_code',anonymous=True)
    rospy.Subscriber('/std_msgs/Float32MultiArray',Float32MultiArray,callback)
    rospy.Subscriber('/camera/depth/image_rect',Image,callback_depth)
    rospy.Subscriber('/co_ords',Float32MultiArray,callback1)
    rospy.spin()
    


if __name__ == '__main__':
    main()
