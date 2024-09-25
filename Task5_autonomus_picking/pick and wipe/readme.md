# object_detection.py :Handles ROS node, subscribers, and publishers

This file will manage ROS communication (i.e., subscribing to the camera feed and publishing the object distances and coordinates).
This script handles ROS initialization, subscribing to the camera feed, and publishing the calculated distances and coordinates.
The callback function receives an image message, converts it to an OpenCV format, and sends it to process_frame from yolo_processing.py.
The processed data is then published, and the frame is displayed.

# yolo_processing.py: Handles YOLO model and image processing
This file will contain the logic for processing frames, running YOLO predictions, and calculating distances.
This script contains the YOLO model processing logic. It processes each frame, detects objects, calculates object distances from the center, and returns the relevant data along with the processed image.
It focuses on object detection, bounding box drawing, and distance calculations.