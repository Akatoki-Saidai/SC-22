import os
import time
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
        self.DEVICENAME                  = '/dev/serial0'

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
    def velocity_control(self,vel):
        print("Press any key to continue! (or press ESC to quit!)")

        # Write goal position
        if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY,vel)
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_VELOCITY, vel)
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

        print("[ID:%03d] GoalVel:%03d  PresVel:%03d" % (self.DXL_ID, self.dxl_goal_velocity, self.dxl_present_position))
        return self.dxl_present_position
    def finish_control(self):
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        # Close port
        self.portHandler.closePort()

if __name__ == '__main__':
    c1 = DynamixelMotor(id_num=1)
    c1.velocity_control()
    c2 = DynamixelMotor(id_num=2)
    c2.velocity_control()
    while True:
        c1.get_prevel()
        time.sleep(2)
        c2.get_prevel()
        time.sleep(2)