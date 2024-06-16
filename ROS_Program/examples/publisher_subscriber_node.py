import rclpy  # ROS2のPythonモジュール
from rclpy.node import Node
from std_msgs.msg import String # トピック通信に使うStringメッセージ型をインポート

class Responder(Node):

   def __init__(self):

       super().__init__('responder') #ノード名の宣言

       self.sub = self.create_subscription(String, '/base_msg', self.msg_callback)# subscriberの宣言　通信の型，トピック名，コールバック関数

       self.pub = self.create_publisher(String,'/ex_msg', 10)# publisherの宣言  メッセージの型，トピック名，キューサイズ

   def msg_callback(self, msg):

       print(msg) # Subscribeしたメッセージをprint
       msg.data = msg.data + "!" # メッセージに「！」を付け加える処理

       self.pub.publish(msg) # 処理したメッセージをPublish

def main():

   rclpy.init()#通信起動

   node = Responder()

   rclpy.spin(node)#ノードを繰り返し実行

   node.destroy_node()#ノード終了

   rclpy.shutdown()#通信終了

if __name__ == '__main__':

   main()