import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading

g_name = str()
g_greet = str()

class NameSub(Node):

   def __init__(self):

       super().__init__('name_sub')

       self.sub = self.create_subscription(String, '/name_msg', self.name_callback)
	

   def name_callback(self, name):

       global g_name

       print(name)
       g_name = name.data	
	
class GreetSub(Node):
	def __init__(self):
       super().__init__('greet_sub')
       self.sub = self.create_subscription(String, '/greet_msg', self.greet_callback)
       self.pub = self.create_publisher(String,'/concat_msg', 10)
	

   def greet_callback(self, greet):

       global g_name, g_greet

       print(greet)
	   g_greet = greet.data
	   msg = String()
	   msg.data = g_name + ", " + g_greet

       self.pub.publish(msg)

def main():

   rclpy.init()

   node1 = NameSub()

   node2 = GreetSub()

   executor = rclpy.executors.MultiThreadedExecutor()

   executor.add_node(node1)

   executor.add_node(node2)

   executor_thread = threading.Thread(target=executor.spin, daemon=True)

   executor_thread.start()

   try:

       while rclpy.ok():

           time.sleep(2)#これなくても良いかも

   except KeyboardInterrupt:

       pass

   rclpy.shutdown()

   executor_thread.join()

if __name__ == '__main__':

   main()