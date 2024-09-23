import cv2
import mediapipe as mp
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Float32


# Initialize ROS node
rospy.init_node('hand_tracking_node')

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

bridge = CvBridge()

# Callback function to process the incoming image messages
def image_callback(msg):
    distance = Float32MultiArray()
    distance.data = (0.0,0.0)
    center = Float32MultiArray()
    center.data = (0.0,0.0)
    norm_center = Float32MultiArray()
    norm_center.data = (0.0,0.0)
    try:
        # Convert ROS image message to OpenCV image
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)
        pub = rospy.Publisher('hand_pose', Float32MultiArray, queue_size=1)
        pub2 = rospy.Publisher('hand_center', Float32MultiArray, queue_size=1)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # Get the bounding box coordinates
                x_min = min([landmark.x for landmark in hand_landmarks.landmark])
                y_min = min([landmark.y for landmark in hand_landmarks.landmark])
                x_max = max([landmark.x for landmark in hand_landmarks.landmark])
                y_max = max([landmark.y for landmark in hand_landmarks.landmark])

                # Convert normalized coordinates to pixel coordinates
                height, width, _ = frame.shape
                x_min, x_max = int(x_min * width), int(x_max * width)
                y_min, y_max = int(y_min * height), int(y_max * height)
                frame_center =  frame.shape[1]//2, frame.shape[0]//2
                center.data = (x_min + x_max) / 2, (y_min + y_max) / 2
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                
                norm_pub  = rospy.Publisher('norm_coords', Float32MultiArray, queue_size=1)
                #normalized value of centerx and centery for depth
                norm_center.data = center_x / width, center_y / height

                
                print("normalized center x and y:", norm_center.data)
                norm_pub.publish(norm_center)
                
                distance_in_pixels = frame_center[0]-center_x, frame_center[1]-center_y
                distance_in_inches = distance_in_pixels[0] * 0.04, distance_in_pixels[1]*0.04 # conversion factor for my camera
                distance.data = distance_in_inches[0] * 2.54, distance_in_inches[1]*2.54
                # print("Distance from frame center:", distance.data, "cm")
                
                pub.publish(distance)
                pub2.publish(center)
                # Draw the bounding box
                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                cv2.putText(frame, f"Distance: {distance.data} cm", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), cv2.FILLED)
                cv2.putText(frame, f"Center: {center.data}", (50,100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                cv2.imshow('Hand Tracking', frame)
                cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

def main():
    # Subscribe to the image topic
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
