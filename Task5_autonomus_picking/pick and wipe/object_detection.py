# #!/usr/bin/env python3

# import cv2, time
# import rospy

# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import numpy as np
# import onnxruntime as ort
# from ultralytics import YOLO
# from std_msgs.msg import Float32MultiArray
# import math as m
# import os
# PPI = 57
# def callback(data):
  
#   global model, bridge, pub , pub1
#   co_ords = Float32MultiArray()
#   co_ords.data = (0.0, 0.0, 0.0, 0.0)
#   distance = Float32MultiArray()
#   distance.data = (0.0,0.0)
#   try:
#     frame = bridge.imgmsg_to_cv2(data, "bgr8")


#   except CvBridgeError as e:
#     print(e)


#   results = model(frame, conf=0.5, iou=0.5, verbose=False)
#   frame_center = (frame.shape[1]//2, frame.shape[0]//2)
#   for cl, box in zip(results[0].boxes.cls, results[0].boxes.xyxyn.cpu()):
   
    
#     if cl == 0.0:


#         co_ords.data = ((box[0]+box[2])/2, (box[1]+box[3])/2, box[0], box[2])
    
        

#   for cl, box in zip(results[0].boxes.cls, results[0].boxes.xyxy.cpu()):
#     if cl == 0.0:
#         # co_ords.data = ((box[0]+box[2])/2, (box[1]+box[3])/2, box[0], box[2])
#         bbox = list(map(int, box.cpu()))
#         object_center = ((bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2)




#         # Calculate the position of the object with respect to the frame center
#         distance_in_pixels = frame_center[0]-object_center[0],frame_center[1]-object_center[1]
        
#         # Convert the distance from pixels to inches
#         distancex_in_inches = distance_in_pixels[0] / PPI , distance_in_pixels[1] / PPI
        
        
#         # Convert the distance from inches to cm
#         distance.data = (distancex_in_inches[0] * 2.54, distancex_in_inches[1] * 2.54)
#     # else:
#     #     os.system('python3 /home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/kortex/api_python/examples/102-Movement_high_level/01-move_angular_and_cartesian_1.py')
        
# #        print("Distance from frame center:", distance.data, "cm")
#         cv2.putText(frame, f"Distance: {distance.data} cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
#         cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 2)
#         cv2.line(frame, (bbox[0], int(bbox[1]*0.5+0.5*bbox[3])), (bbox[2], int(bbox[1]*0.5+0.5*bbox[3])), (0, 0, 255), thickness=2)
#         cv2.line(frame, (int(bbox[0]*0.5+0.5*bbox[2]), bbox[1]), (int(bbox[0]*0.5+0.5*bbox[2]), bbox[3]), (0, 0, 255), thickness=2)
        

#   cv2.line(frame, (640, 0), (640, 720), (0, 255, 0), thickness=2)
#   cv2.line(frame, (0, 360), (1280, 360), (0, 255, 0), thickness=2)


#   pub.publish(distance)
#   pub1.publish(co_ords)
#   cv2.imshow("Image", frame)
#   cv2.waitKey(1)



# def main():
  
#   global model, bridge, pub, COUNTER, pub1

#   COUNTER = 0

#   bridge = CvBridge()
    
#   rospy.init_node('my_code', anonymous=True)

#   model = YOLO("/home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics/dynamic_moping/mop.pt")
  
#   image_sub = rospy.Subscriber('/camera/color/image_raw', Image, callback)
#   pub = rospy.Publisher('/std_msgs/Float32MultiArray', Float32MultiArray, queue_size=10)
#   pub1 = rospy.Publisher('co_ords', Float32MultiArray, queue_size=10)

#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")

#   cv2.destroyAllWindows()


# if __name__ == '__main__':

#   main()



#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray

PPI = 57

def callback(data):
    global model, bridge, pub, pub1 
    co_ords = Float32MultiArray()
    co_ords.data = (0.0, 0.0, 0.0, 0.0)
    distance = Float32MultiArray()
    distance.data = (0.0, 0.0)

    try:
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

    results = model(frame, conf=0.5, iou=0.5, verbose=False)
    frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)

    for cl, box, conf in zip(results[0].boxes.cls, results[0].boxes.xyxyn.cpu(), results[0].boxes.conf.cpu()):
        if cl == 0.0 and conf > 0.7:
            co_ords.data = ((box[0] + box[2]) / 2, (box[1] + box[3]) / 2, box[0], box[2])

    for cl, box, conf in zip(results[0].boxes.cls, results[0].boxes.xyxy.cpu(), results[0].boxes.conf.cpu()):
        if cl == 0.0 and conf > 0.7:
            bbox = list(map(int, box.cpu()))
            object_center = ((bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2)

            # Calculate the position of the object with respect to the frame center
            distance_in_pixels = frame_center[0] - object_center[0], frame_center[1] - object_center[1]

            # Convert the distance from pixels to inches
            distancex_in_inches = distance_in_pixels[0] / PPI, distance_in_pixels[1] / PPI

            # Convert the distance from inches to cm
            distance.data = (distancex_in_inches[0] * 2.54, distancex_in_inches[1] * 2.54)

            cv2.putText(frame, f"Distance: {distance.data} cm", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 2)
            cv2.line(frame, (bbox[0], int(bbox[1] * 0.5 + 0.5 * bbox[3])), (bbox[2], int(bbox[1] * 0.5 + 0.5 * bbox[3])), (0, 0, 255), thickness=2)
            cv2.line(frame, (int(bbox[0] * 0.5 + 0.5 * bbox[2]), bbox[1]), (int(bbox[0] * 0.5 + 0.5 * bbox[2]), bbox[3]), (0, 0, 255), thickness=2)

    cv2.line(frame, (640, 0), (640, 720), (0, 255, 0), thickness=2)
    cv2.line(frame, (0, 360), (1280, 360), (0, 255, 0), thickness=2)

    pub.publish(distance)
    pub1.publish(co_ords)
    cv2.imshow("Image", frame)
    cv2.waitKey(1)

def main():
    global model, bridge, pub, COUNTER, pub1

    COUNTER = 0
    bridge = CvBridge()

    rospy.init_node('my_code', anonymous=True)

    model = YOLO("/home/architsharma/catkin_workspace/src/ros_kortex/kortex_examples/src/move_it/healthcare/center_based_reach/final_inverse_kinematics/dynamic_moping/mop.pt")

    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, callback)
    pub = rospy.Publisher('/std_msgs/Float32MultiArray', Float32MultiArray, queue_size=10)
    pub1 = rospy.Publisher('co_ords', Float32MultiArray, queue_size=10)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
