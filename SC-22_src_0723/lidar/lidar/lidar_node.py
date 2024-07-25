import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import gpiozero
import time

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'lidar_link'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 1.0
        transform_stamped.transform.translation.z = 1.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 0.0
        broadcaster = StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform_stamped)
        """
        ECHO = 18
        self.echo = gpiozero.DigitalInputDevice(ECHO)
        self.publisher_lidar = self.create_publisher(LaserScan,'/scan',10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.lidar_callback)   

    def lidar_callback(self):
        msg_lidar = LaserScan()
        msg_lidar.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg_lidar.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg_lidar.header.frame_id = "base_link"
        msg_lidar.angle_min = 0
        msg_lidar.angle_min = 3.14159
        msg_lidar.angle_increment = 3.14159/180
        msg_lidar.time_increment  = 0.0002
        msg_lidar.range_min = 0.015
        msg_lidar.range_max = 0.640
        time.sleep(0.0002)
        data = []
        for i in range(0,180):
            while self.echo.is_active == False:
                pulse_start = time.time()

            while self.echo.is_active == True:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start

            distance = 34300 * (pulse_duration/2)/1000
            msg_lidar.scan_time = pulse_duration
            data.append(distance)
        msg_lidar.ranges = data
        self.publisher_lidar.publish(msg_lidar)

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


