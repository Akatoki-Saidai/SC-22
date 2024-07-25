import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped5
from micropygps import MicropyGPS
import pyserial

class GNSSNode(Node):
    def __init__(self):
        super().__init__('gnss_node')
        
        self.uart = serial.Serial('/dev/ttyAMA0',9600,timeout=0.5)
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'gnss'
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
        self.publisher_gnss = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.gnss_callback)        
        
    def gnss_callback(self):
        # データ取得
        msg_gnss = NavSatFix()

        msg_gnss.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg_gnss.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg_gnss.header.frame_id = "base_link"
        gnss_data = self.getData()
        msg_gnss.latitude = gnss_data.latitude[0]
        msg_gnss.longitude = gnss_data.longitude[0]
        self.publisher_gnss.publish(msg_gnss)
    
    def getData(self):
        # gps設定
        my_gnss = MicropyGPS(9, 'dd')

        # 10秒ごとに表示
        tm_last = 0
        count = 0
        (count, sentence) = self.uart.readline()
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    stat = my_gnss.update(chr(x))
                    if stat:
                        tm = my_gnss.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            #print('=' * 20)
                            #print(my_gnss.date_string(), tm[0], tm[1], int(tm[2]))
                            #print("latitude:", my_gnss.latitude[0], ", longitude:", my_gnss.longitude[0])
                            return my_gnss


def main(args=None):
    rclpy.init(args=args)
    gnss_node = GNSSNode()
    rclpy.spin(gnss_node)
    gnss_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

