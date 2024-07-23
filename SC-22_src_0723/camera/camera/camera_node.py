import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'map'
        transform_stamped.child_frame_id = 'camera'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 1.0
        transform_stamped.transform.translation.z = 1.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 0.0
        broadcaster = StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform_stamped)
        self.publisher = self.create_publisher(Image, '/image', 10)
        self.bridge = cv_bridge.CvBridge()
        self.camera = cv2.VideoCapture(0)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.image_callback)

    def image_callback(self):
        # OpenCV の様式に変更
        ret,frame = self.camera.read()
        if ret == True:
        # ROSの様式に変換
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp.sec = self.get_clock().now().to_msg().sec
            msg.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
            msg.header.frame_id = "camera"
            # トピックとして配信する
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()