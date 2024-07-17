from math import *
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_lib import euler_from_quaternion, quaternion_from_euler
import numpy
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
class CansatOdom():
    def __init__(self):
        self.save_path_csv = True
        self.subscribe_cmd_vel = True
        # d = 140mm,65rpm
        self.Max_SPEED_KMH = 0.476475
        self.MIN_SPEED_KMH = -0.476475
        
        initPosx = 0.0
        initPosy = 0.0

        self.odom_header = Header()
        self.odom_header.frame_id = "map"

        self.pose = Pose()
        self.pose.position.x = initPosx
        self.pose.position.y = initPosy
        self.pose.position.z = 0.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.twist = Twist()

        # Initialize odometry info
        self.odom = Odometry()
        self.odom.header = self.odom_header
        self.odom.child_frame_id = "base_link"
        self.odom.pose.pose = self.pose
        self.odom.twist.twist = self.twist
        self.publisher_odom = self.create_publisher(Odometry, '/odom', 10)

        self.rate = 10.0
        self.t_delta = rclpy.Duration(1.0/self.rate)
        self.t_next = rclpy.Time.now() + self.t_delta
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rclpy.Time.now() 
        self.odomBroadcaster = TransformBroadcaster()   

        self.base_width = 0.140

        self.sub_vel_right = Subscriber(self,Float32,'/vel_right')
        self.sub_vel_left = Subscriber(self,Float32,'/vel_left')
        # ApproximateTimeSynchronizer with queue size 10 and 0.1 seconds slop
        self.ts = ApproximateTimeSynchronizer([self.sub_vel_right, self.sub_vel_left], 10, 0.1)
        self.ts.registerCallback(self.update_odom)

    def update_odom(self,v_right,v_left):
        now = rclpy.Time.now()
        if now > self.t_next:
            elapsed = now-self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            # distance traveled is the average of the two wheels
            d = ( v_left + v_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( v_right - v_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            self.pose.position.x = self.x
            self.pose.position.y = self.y

            self.pose.orientation.x = 0.0
            self.pose.orientation.y = 0.0
            self.pose.orientation.z = sin(self.th/2)
            self.pose.orientation.w = cos(self.th/2)
            self.twist.linear.x = cos(self.th)*(v_left+v_right)/2
            self.twist.linear.y = sin(self.th)*(v_left+v_right)/2
            self.twist.linear.z = 0
            self.twist.angular.x = cos(self.th)
            self.twist.angular.y = sin(self.th)
            self.twist.angluar.z = 0
            self.odom.header = self.odom_header
            self.odom.pose.pose = self.pose
            self.odom.twist.twist = self.twist
            self.publisher_odom.publish(self.odom)

            map_frame = TransformStamped()
            map_frame.header.frame_id = 'map'
            map_frame.child_frame_id = 'odom'
            map_frame.transform.translation.x = self.pose.position.x
            map_frame.transform.translation.y = self.pose.position.y
            map_frame.transform.translation.z = 0.0
            map_frame.transform.rotation.w = self.pose.orientation.w
            map_frame.transform.rotation.x = self.pose.orientation.x
            map_frame.transform.rotation.y = self.pose.orientation.y
            map_frame.transform.rotation.z = self.pose.orientation.z

            self.odomBroadcaster.sendTransform(map_frame)

