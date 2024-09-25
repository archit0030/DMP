import cv2
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32MultiArray

# Load YOLO model (you can modify this path)
model = YOLO("mop.pt") 

# Function to process a single frame with the YOLO model
def process_frame(frame, PPI):
    co_ords = Float32MultiArray()
    co_ords.data = (0.0, 0.0, 0.0, 0.0)
    distance = Float32MultiArray()
    distance.data = (0.0, 0.0)

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

            # Draw bounding box, center lines, and distance text on the frame
            cv2.putText(frame, f"Distance: {distance.data} cm", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 0, 255), 2)
            cv2.line(frame, (bbox[0], int(bbox[1] * 0.5 + 0.5 * bbox[3])), (bbox[2], int(bbox[1] * 0.5 + 0.5 * bbox[3])), (0, 0, 255), thickness=2)
            cv2.line(frame, (int(bbox[0] * 0.5 + 0.5 * bbox[2]), bbox[1]), (int(bbox[0] * 0.5 + 0.5 * bbox[2]), bbox[3]), (0, 0, 255), thickness=2)

    # Draw center lines for the frame
    cv2.line(frame, (640, 0), (640, 720), (0, 255, 0), thickness=2)
    cv2.line(frame, (0, 360), (1280, 360), (0, 255, 0), thickness=2)

    return co_ords, distance, frame
