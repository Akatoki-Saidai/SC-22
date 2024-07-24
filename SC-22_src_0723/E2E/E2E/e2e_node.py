import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,FluidPressure,IMU,MagneticField,Temperature,NavSatFix
from std_msgs.msg import Int16,Float32
from message_filters import ApproximateTimeSynchronizer, Subscriber
from math import *
import smach
class FallingPhase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['longphase'])
        self.subscriber_fluid = Subscriber(self,FluidPressure,'/atomsphere')
        self.subscriber_imu = Subscriber(self,IMU,'/imu')
        # ApproximateTimeSynchronizer with queue size 10 and 0.1 seconds slop
        self.ts = ApproximateTimeSynchronizer([self.subscriber_fluid, self.subscriber_imu], 10, 0.1)
        self.ts.registerCallback(self.execute)
        self.init_pressure = 1008
    def execute(self,fluid,imu):
        if fluid < self.init_pressure + 10:
            return 'longpahse'
        else:
            self.get_logger().info("Falling Now")
class LongPhase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['shortphase'])
        self.sub_north_degree = Subscriber(self,Float32,'/mag')
        self.sub_gps = Subscriber(self,NavSatFix,'/gps')
        # ApproximateTimeSynchronizer with queue size 10 and 0.1 seconds slop
        self.ts = ApproximateTimeSynchronizer([self.sub_north_degree, self.sub_gps], 10, 0.1)
        self.ts.registerCallback(self.degree_from_front_to_goal)

        self.publisher_north_degree = self.create_publisher(Float32, '/north_degree', 10)
        self.publisher_goal_degree = self.create_publisher(Float32, '/goal_degree', 10)
        self.publisher_goal_distance = self.create_publisher(Float32, '/goal_distance', 10)
        self.subscriber_north_degree = self.create_subscription(MagneticField,'/mag',self.calculate_north_degree,10)
        self.subscriber_goal_degree = self.create_subscription(NavSatFix,'/gps',self.degree_from_front_to_goal,10)
        self.subscriber_goal_distance = self.create_subscription(NavSatFix,'/gps',self.distance,10)

        self.sub_goal_degree = Subscriber(self,Float32,'/goal_degree')
        self.sub_goal_distance = Subscriber(self,Float32,'/goal_distance')
        # ApproximateTimeSynchronizer with queue size 10 and 0.1 seconds slop
        self.ts = ApproximateTimeSynchronizer([self.sub_goal_degree, self.sub_goal_distance], 10, 0.1)
        self.ts.registerCallback(self.execute)

        self.goal_latitude = 0
        self.goal_longitude  =  0
        self.radius = 6378.137
    def execute(self,degree,distance):
        if distance < 10:
            return 'shortphase'
        else:
            #degreeを0にする使ったモータの動きを記述
            a
    def calculate_north_degree(self,msg):

        mx = msg.magnetic_field.x    
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z
        # Calculate magnetic strength and direction.
        magnitude = sqrt(mx * mx + my * my + mz * mz)
        if my > 0:
            direction = atan2(mx, my) * 180 / pi
        elif my < 0:
            direction = (atan2(mx, my) + 2 * pi) * 180 / pi
        else:
            if mx < 0:
                direction = 270.0
            else:
                direction = 90.0
        # Convert direction to heading.
        heading = Float32()
        heading.data = 360 - direction +90
        if heading.data >= 360:
            heading.data -= 360
        if heading.data < 0:
            heading.data += 360
        #print("north_degree",heading)
        self.publisher_north_degree.publish(heading)
    
    def azimuth(self,a,b,ap,bp):
        #a = lat, b = long, ap = latp ,bp = longp
        ag = self.goal_latitude
        bg = self.goal_longitude
        fai1  = atan2(sin(b - bp),cos(bp)*tan(b) - sin(bp)*cos(a - ap))
        fai2 =  atan2(sin(b - bg),cos(bg)*tan(b) - sin(bg)*cos(a - ag))
        dfai = fai2 - fai1 #回転する角度
        return dfai
    
    def degree_from_front_to_goal(self,north_deg,gps):
        now_lat = gps.latitude
        now_lon = gps.longitude
        north_degree = north_deg.data
        GPS_degree = self.azimuth(self.goal_latitude,self.goal_longitude,now_lat,now_lon)
        degree = Float32()
        degree.data = GPS_degree - north_degree
        self.publisher_goal_degree.publish(degree)

    def distance(self,msg):
        d = Float32()
        a = self.goal_latitude
        b = self.goal_longitude
        ap = msg.latitude
        bp = msg.logitude
        r = self.radius
        d.data = r*acos(sin(pi/180*ap)*sin(pi/180*a)+cos(pi/180*ap)*cos(pi/180*a)*cos(pi/180*b-pi/180*bp))
        self.publisher_goal_distance.publish(d)
class ShortPhase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finishphase'])
        self.subscriber_goal_center = self.create_subscription(Int16,'/goal_center',self.execute,10)
    def execute(self,msg):
        if 0 < msg.data < 100:
            #ゴールに向かうモータの動きを記述
            a
        elif 101 < msg.data < 200:
            pass

class FinishPhase(smach.State):
    def __init__(self):
        self.get_logger().info("Finish")

        
def main():
    rclpy.init()
    node = Node("state_machine")
    sm = smach.StateMachine(outcomes=['finishphase'])
    with sm:
        smach.StateMachine.add('fallingphase',FallingPhase(),transitions={'longphase' : 'LongPhase','shortphase' : 'ShortPhase'})
        smach.StateMachine.add('longphase',LongPhase(),transitions={'shortphase' : 'ShortPhase'})
        smach.StateMachine.add('shortphase',ShortPhase(),transitions={'finishphase' : 'FinishPhase'})
    outcome = sm.execute()

if __name__ == "__main__":
    main()