import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Responder2(Node):

   def __init__(self):

       super().__init__('responder2')

       self.sub1 = self.create_subscription(String, '/name_msg', self.name_callback)
       self.sub2 = self.create_subscription(String, '/greet_msg', self.greet_callback)

       self.pub = self.create_publisher(String,'/concat_msg', 10)

       self.name = str()
       self.greet = str()

   def name_callback(self, name):

       print(name)
       self.name = name.data
	

   def greet_callback(self, greet):

       print(greet)
       self.greet = greet.data
       msg = String()
       msg.data = self.name + ", " + self.greet

       self.pub.publish(msg)

def main():

   rclpy.init()

   node = Responder2()

   rclpy.spin(node)

   node.destroy_node()

   rclpy.shutdown()

if __name__ == '__main__':

   main()