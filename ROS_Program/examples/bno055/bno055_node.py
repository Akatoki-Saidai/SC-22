import rclpy
from rclpy.node import Node
from sensor_msgs.msg import IMU,MagneticField,Temperature
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped,Vector3
from Adafruit_BNO055 import BNO055

class Bno055Node(Node):
    def __init__(self):
        super().__init__('bno055_node')
        
        self.bno = BNO055.BNO055(rst=18)

        if not bno.begin():
            raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'bno055'
        transform_stamped.transform.translation.x = 0.0
        transform_stamped.transform.translation.y = 1.0
        transform_stamped.transform.translation.z = 1.0
        transform_stamped.transform.rotation.x = 0.0
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 0.0
        broadcaster = StaticTransformBroadcaster(self)
        broadcaster.sendTransform(transform_stamped)
        self.publisher_imu = self.create_publisher(IMU, '/quat', 10)
        self.publisher_mag = self.create_publisher(MagneticField, '/mag', 10)
        self.publisher_ang = self.create_publisher(Vector3,'/vec3',10)
        self.publisher_temp = self.create_publisher(Temperature,'/temp',10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.IMU_callback)        
        
    def IMU_callback(self):
        # データ取得
        msg_imu = IMU()
        msg_mag = MagneticField()
        msg_ang = Vector3()
        msg_tmp = Temperature()
        heading, roll, pitch = self.bno.read_euler()
        #sys, gyro, accel, mag = bno.get_calibration_status() 
        qx,qy,qz,qw = self.bno.read_quaterion()        
        temp_c = self.bno.read_temp()        
        mx,my,mz = self.bno.read_magnetometer()       
        Gx,Gy,Gz = self.bno.read_gyroscope()       
        #ax,ay,az = bno.read_accelerometer()       
        lx,ly,lz = self.bno.read_linear_acceleration()        
        #gx,gy,gz = bno.read_gravity()   
        msg_imu.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg_imu.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg_imu.header.frame_id = "bno055"
        msg_mag.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg_mag.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg_mag.header.frame_id = "bno055"
        msg_tmp.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg_tmp.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg_tmp.header.frame_id = "bno055"
        msg_ang.header.stamp.sec = self.get_clock().now().to_msg().sec
        msg_ang.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        msg_ang.header.frame_id = "bno055"

        msg_imu.orientation.x = qx
        msg_imu.orientation.y = qy
        msg_imu.orientation.z = qz
        msg_imu.orientation.w = qw
        msg_imu.angular_velocity.x = Gx
        msg_imu.angular_velocity.y = Gy
        msg_imu.angular_velocity.z = Gz
        msg_imu.linear_acceleration.x = lx
        msg_imu.linear_acceleration.y = ly
        msg_imu.linear_acceleration.z = lz
        self.publisher_imu.publish(msg_imu)

        msg_mag.magnetic_field.x = mx
        msg_mag.magnetic_field.y = my
        msg_mag.magnetic_field.z = mz
        self.pub_mag.publish(msg_mag)

        msg_tmp.temperature = temp_c
        self.publisher_temp.publish(msg_tmp)
        
        msg_ang.x = roll
        msg_ang.y = pitch
        msg_ang.z = heading
        self.publisher_ang.publish(msg_ang)
def main(args=None):
    rclpy.init(args=args)
    bno055_node = Bno055Node()
    rclpy.spin(bno055_node)
    bno055_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

