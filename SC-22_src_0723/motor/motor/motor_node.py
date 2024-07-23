from rclpy.node import Node
from std_msgs.msg import Float32,JointState
from message_filters import ApproximateTimeSynchronizer, Subscriber
import dynamixel_motor
class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')
        self.publisher_R = self.create_publisher(JointState, '/rwheel/data', 10)
        self.publisher_L = self.create_publisher(JointState, '/lwheel/data', 10)
        self.subscriber_wheel_R = Subscriber(self,Float32,'/rwheel/vel')
        self.subscriber_wheel_L = Subscriber(self,Float32,'lwheel/vel')
        self.motor_R = DynamixelMotor(id_num=1)
        self.motor_L = DynamixelMotor(id_num=2)
        # ApproximateTimeSynchronizer with queue size 10 and 0.1 seconds slop
        self.ts = ApproximateTimeSynchronizer([self.subscriber_wheel_R, self.subscriber_wheel_L], 10, 0.01)
        self.ts.registerCallback(self.control_motor)



    def control_motor(self,vel_R,vel_L):
        self.motor_R.velocity_control(vel_R.data)
        self.motor_L.velocity_control(vel_L.data)
        prevel_R = JointState()
        prevel_L = JointState()
        prevel_R.header.frame_id = "Rwheel_link"
        prevel_L.header.frame_id = "Lwheel_link"
        prevel_R.header.name = "r_joint"
        prevel_L.header.name = "l_joint"
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