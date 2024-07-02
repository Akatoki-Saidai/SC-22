from rclpy.node import Node
from std_msgs.msg import Float32,JointState
import dynamixel_motor
class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')
        self.publisher_R = self.create_publisher(JointState, '/wheel_R', 10)
        self.publisher_L = self.create_publisher(JointState, '/wheel_L', 10)
        self.subscriber = self.create_subscription(Float32,'/velocity',self.control_motor,10)
        self.motor_R = DynamixelMotor(id_num=1)
        self.motor_L = DynamixelMotor(id_num=2)


    def control_motor(self,vel_R,vel_L):
        self.motor_R.velocity_control(vel_R)
        self.motor_L.velocity_control(vel_L)
        prevel_R = JointState()
        prevel_L = JointState()
        prevel_R.header.frame_id = "wheel_R"
        prevel_L.header.frame_id = "wheel_L"
        prevel_R.header.stamp.sec = self.get_clock().now().to_msg().sec
        prevel_R.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        prevel_L.header.stamp.sec = self.get_clock().now().to_msg().sec
        prevel_L.header.stamp.nanosec = self.get_clock().now().to_msg().nanosec
        prevel_R.velocity = self.motor_R.get_prevel()
        prevel_L.velocity = self.motor_L.get_prevel()
        self.publisher_R.publish(prevel_R)
        self.publisher_R.publish(prevel_R)
def main():
    rclpy.init()
    motor_node = MotorNode()
    rclpy.spin(motor_node)
    motor_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()