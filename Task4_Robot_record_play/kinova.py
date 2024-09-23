#!/usr/bin/env python3

import cv2
import rospy
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os
import datetime
import time
PPI = 57

frame_count = 0

def callback(data):
    global frame, out, frame_count
    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")

        # Resize frame to 640x480 pixels
        frame = cv2.resize(frame, (640, 480))

        
        pub = rospy.Publisher("xyz", Image, queue_size=1)
        pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
        
        frame_count += 1
        out.write(frame)
        cv2.putText(frame, "Frame: {}".format(frame_count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if frame_count == 1200:
            # Write frame to video if under 1000 frames
            
            rospy.signal_shutdown("Reached 1000 frames")

    except CvBridgeError as e:
        print(e)

    cv2.imshow("Image", frame)
    
    cv2.waitKey(1)


def main():
    global bridge, frame, out, frame_count

    bridge = CvBridge()

    rospy.init_node("Color_Image", anonymous=True)
    time.sleep(1.8)

    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, callback)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_name1 = datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + "_kinova.avi"
    out = cv2.VideoWriter(video_name1, cv2.VideoWriter_fourcc(*"XVID"), 30, (640, 480))

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # Release video writer and close OpenCV windows
    out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
