from bme280 import bme280
from bme280 import bme280_i2c
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
class Bme280Node(Node):
    def __init__(self):
        super().__init__('bme280_node')
        # 初期化
        bme280_i2c.set_default_i2c_address(0x76)
        bme280_i2c.set_default_bus(1)
        # キャリブレーション
        bme280.setup()
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'bme280'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 1.0
        transform_stamped.transform.translation.z = 1.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 0.0
        broadcaster = StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform_stamped)
        self.publisher = self.create_publisher(FluidPressure, '/pressure', 10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.pressure_callback)        
        
    def pressure_callback(self):
        # データ取得
        msg = FluidPressure()
        data_all = bme280.read_all()
        msg = data_all.pressure
        self.publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    bme280_node = Bme280Node()
    rclpy.spin(bme280_node)
    bme280_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
