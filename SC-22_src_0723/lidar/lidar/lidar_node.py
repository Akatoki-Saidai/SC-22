import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import gpiozero
import time

import dynamixel_motor

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

class DynamixelMotor():
    def __init__(self,id_num):
        #********* DYNAMIXEL Model definition *********
        #***** (Use only one definition at a time) *****
        self.MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
        # self.MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
        # self.MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
        # self.MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
        # self.MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
        # self.MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


        # Control table address
        if self.MY_DXL == 'X_SERIES' or self.MY_DXL == 'MX_SERIES':
            self.ADDR_TORQUE_ENABLE          = 64
            self.ADDR_GOAL_POSITION          = 116
            self.ADDR_PRESENT_POSITION       = 132
            self.ADDR_GOAL_VELOCITY          = 104
            self.ADDR_PRESENT_VELOCITY       = 128
            self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
            self.DXL_MAXIMUM_POSITION_VALUE  = 4095 
            self.DXL_MINIMUM_VELOCITY_VALUE  = 0
            self.DXL_MAXIMUM_VELOCITY_VALUE  = 300   
                # Refer to the Maximum Position Limit of product eManual
            self.BAUDRATE                    = 57600
        elif self.MY_DXL == 'PRO_SERIES':
            self.ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
            self.ADDR_GOAL_POSITION          = 596
            self.ADDR_PRESENT_POSITION       = 611
            self.DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
            self.DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
            self.BAUDRATE                    = 57600
        elif self.MY_DXL == 'P_SERIES' or self.MY_DXL == 'PRO_A_SERIES':
            self.ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
            self.ADDR_GOAL_POSITION          = 564
            self.ADDR_PRESENT_POSITION       = 580
            self.DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
            self.DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
            self.BAUDRATE                    = 57600
        elif self.MY_DXL == 'XL320':
            self.ADDR_TORQUE_ENABLE          = 24
            self.ADDR_GOAL_POSITION          = 30
            self.ADDR_PRESENT_POSITION       = 37
            self.DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
            self.DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
            self.BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

        # DYNAMIXEL Protocol Version (1.0 / 2.0)
        # https://emanual.robotis.com/docs/en/dxl/protocol2/
        self.PROTOCOL_VERSION            = 2.0

        # Factory default ID of all DYNAMIXEL is 1
        self.DXL_ID                      = id_num

        # Use the actual port assigned to the U2D2.
        # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
        self.DEVICENAME                  = '/dev/ttyAMA0'

        self.TORQUE_ENABLE               = 1     # Value for enabling the torque
        self.TORQUE_DISABLE              = 0     # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

        self.dxl_goal_velocity = self.DXL_MAXIMUM_VELOCITY_VALUE        # Goal position


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")
    def position_control(self,value):
        print("Press any key to continue! (or press ESC to quit!)")

        # Write goal position
        if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, value/0.088)
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION,value/0.088)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % 0.088*self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % 0.088*self.packetHandler.getRxPacketError(dxl_error))
    def get_prepos(self):
        # Read present position
        if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            self.dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        else:
            self.dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] PresPos:%03d deg" % (self.DXL_ID, 0.088*self.dxl_present_position))
        return 0.088*self.dxl_present_position
    
    def velocity_control(self,value):
        print("Press any key to continue! (or press ESC to quit!)")

        # Write goal position
        if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, value/0.229)
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, value/0.229)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    def get_prevel(self):
        # Read present position
        if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            self.dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        else:
            self.dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_VELOCITY)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] PresVel:%03d rpm" % (self.DXL_ID,0.229*self.dxl_present_position))
        return 0.229*self.dxl_present_position

    def finish_control(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()

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
        self.motor = DynamixelMotor(id_num = 1)

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
            self.motor.position_control(i)
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


