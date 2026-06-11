# Collection of YOLOv8 detections associated with a single image frame.
#
# The header timestamp can be used to synchronize detections with
# camera images or other sensor data.
#
# Each element in 'yolov8_inference' represents one detected object
# and its corresponding bounding box.

std_msgs/Header header

InferenceResult[] yolov8_inference