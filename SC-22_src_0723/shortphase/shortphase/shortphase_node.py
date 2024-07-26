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
        if goal:
            self.get_logger().info("Finish")
            return 'finishphase'
