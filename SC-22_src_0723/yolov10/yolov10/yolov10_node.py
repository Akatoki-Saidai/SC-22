import supervision as sv
from ultralytics import YOLOv10
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
import cv2
import cv_bridge

class Yolov10Node(Node):

    def __init__(self):
        super().__init__('yolov10_node')
        self.publisher = self.create_publisher(Image, '/image_annotated/data', 10)
        self.publisher_xyxy = self.create_publisher(Int16, '/cone_position/data', 10)
        self.subscriber = self.create_subscription(Image,'/image/data',self.process_yolov10,10)
        self.model = YOLOv10("/home/konan/ros2_ws/src/yolov10/yolov10/best.pt")
        self.bridge = cv_bridge.CvBridge()
        self.cone_position = Int16()

    def process_yolov10(self,frame):
        frame = self.bridge.imgmsg_to_cv2(frame, "bgr8")
        bounding_box_annotator = sv.BoundingBoxAnnotator()
        label_annotator = sv.LabelAnnotator()
        results = self.model(frame,conf=0.70)[0]
        if len(results.boxes.xyxy) > 0:
            self.cone_position.data = int((results.boxes.xyxy[0][0]+results.boxes.xyxy[0][2])/2)
            self.publisher_xyxy.publish(self.cone_position)
        #print(np.argmax(results.boxes.conf))
        detections = sv.Detections.from_ultralytics(results)
        annotated_frame = bounding_box_annotator.annotate(scene=frame, detections=detections)
        annotated_frame = label_annotator.annotate(scene=annotated_frame, detections=detections)
        msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
        msg.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg.header.frame_id = "camera_anotated"
        self.publisher.publish(msg)
        
def main():
    rclpy.init()
    yolov10_node = Yolov10Node()
    rclpy.spin(yolov10_node)
    yolov10_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()