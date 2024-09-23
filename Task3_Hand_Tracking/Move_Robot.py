#!usr/bin/env python3
import rospy
from kortex_driver.msg import TwistCommand
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32 , Empty, Float32MultiArray
from sensor_msgs.msg import Image
import subprocess
import os

kp = 0.8
def callback(data):
    ggg = data.data
    msg= ggg[0]*kp, ggg[1]*kp
    print(msg[0])
    rate = rospy.Rate(1) # 10hz
    pub = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=10)


    my = TwistCommand()
    my.reference_frame  = 0
    zx = Twist()
    my.duration = 1
    # for y axis
    if msg[0] <= 29 and msg[0] >= 8:
        my.twist.linear_y = 0.05
    elif msg[0] >= -29 and msg[0] <= -8:
        my.twist.linear_y = -0.05
        
    elif msg[0] <8 and msg[0] >= 3:
        my.twist.linear_y = 0.02   
    elif msg[0] > -8 and msg[0] <= -3:
        my.twist.linear_y = -0.02 
        
    elif msg[0] <3 and msg[0] >= 0.2:
        my.twist.linear_y = 0.02
    elif msg[0] > -3 and msg[0] <= -0.2:
        my.twist.linear_y = -0.02
        
    elif msg[0] <0.2 and msg[0] > -0.2:
        my.twist.linear_y = 0

            
            
    #for z axis   
    if msg[1]<= 29 and msg[1]>= 8:
        my.twist.linear_z = 0.03
    elif msg[1]>= -29 and msg[1]<= -8:
        my.twist.linear_z = -0.03
    elif msg[1]<8 and msg[1]>= 3:
        my.twist.linear_z = 0.01
        
    elif msg[1]> -8 and msg[1]<= -3:
        my.twist.linear_z = -0.01
    elif msg[1]<3 and msg[1]>= 0.2:
        my.twist.linear_z = 0.01
    elif msg[1]> -3 and msg[1]<= -0.2:
        my.twist.linear_z = -0.01
    elif msg[1]<0.2 and msg[1]> -0.2:
        my.twist.linear_z = 0
    else:
        print("Value out of range")
        
        
    pub.publish(my) 

def main():
    rospy.init_node('cartesian_publisher',anonymous=True)
    rospy.Subscriber('/hand_pose',Float32MultiArray,callback)
    rospy.spin()

if __name__ == '__main__':
    main()
