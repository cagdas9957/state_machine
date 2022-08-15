#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import smach_ros
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatusArray


class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])
    def execute(self,ud):
        rospy.sleep(1)
        return 'success'

class SensorCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['checked','failed'])
    def execute(self, ud):
        rospy.sleep(100)
        a=True
        if a==True:
            return 'checked'
        else:
            return 'failed'

class Pos(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['check','exit'],
                            output_keys=['counter'])
        self.counter=0
        self.pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

        rospy.loginfo(self.counter)

    def execute(self,userdata):
        self.counter+=1
        userdata.counter=self.counter
        print("KANTIR: ", self.counter)
        if self.counter<=3:

            rospy.loginfo("enter x pos:")
            self.pos_x=int(input() or 2.0)
            rospy.loginfo("enter y pos:")
            self.pos_y=int(input() or 2.0)
            rospy.loginfo("enter w pos:")
            self.pos_w=int(input() or 1.0)

            goal = PoseStamped()
            goal.header.stamp=rospy.Time.now()
            goal.header.frame_id = "odom"
            goal.pose.position.x = self.pos_x
            goal.pose.position.y = self.pos_y
            goal.pose.orientation.w = self.pos_w
           
            self.pub.publish(goal)

            rospy.loginfo('DESTINATION DETERMINED')
            rospy.loginfo('CHECK STATE')
            return 'check'
        else:
            rospy.loginfo('MOVE COMPLETE')
            return 'exit'

class Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['pos','exit'],
                            input_keys=['counter'])
        #For simulation                    
        rospy.Subscriber('/locomove_base/status',GoalStatusArray,self.control) 
        #For real                    
        # rospy.Subscriber('/move_base/status',Bool,self.control_real)

        self.check=None

    def control(self,data):
        self.check=data.status_list[0].status

    def control_real(self,data):
        self.check=data.data

    def execute(self, userdata):

        rate = rospy.Rate(10)
        while(self.check==None):
            rate.sleep()

        if self.check==3 and userdata.counter<=3: #self.check simülasyonda 3 olmalı
            rate.sleep()
            rospy.loginfo('POS STATE')
            self.check=None
            return 'pos'
        elif self.check==3 and userdata.counter == 4:
            rospy.loginfo('MOVE COMPLETE')
            self.check=None
            return 'exit'
    
def main():

    
    rospy.init_node('machina',anonymous=True)

    sm=smach.StateMachine(outcomes=['FINISH','FAILED'])
    

    with sm:

        smach.StateMachine.add('INIT',Init(),
                transitions={'success':'CON'})

        sm_con=smach.Concurrence(outcomes = ['exit','fail'],
                default_outcome='exit',
                outcome_map={'exit':
                            {'SM_MAIN':'FINISHED'},
                            'fail': 
                            {'SM_SENSOR':'FAILED'}})
        
        sm_sensor=smach.StateMachine(outcomes=['FAILED'])

        with sm_sensor:
            smach.StateMachine.add('SENSOR',SensorCheck(),
                                    transitions={'checked':'SENSOR','failed':'FAILED'})

        sm_main=smach.StateMachine(outcomes=['FINISHED'])

        with sm_main:
            smach.StateMachine.add('POS',Pos(),
                transitions={'check':'CHECK','exit':'FINISHED'},
                remapping={'counter':'sm_data'})

            smach.StateMachine.add('CHECK',Check(),
                transitions={'pos':'POS','exit':'FINISHED'},
                remapping={'counter':'sm_data'})


        with sm_con:

            smach.Concurrence.add('SM_SENSOR',sm_sensor)

            smach.Concurrence.add('SM_MAIN',sm_main)
            
        smach.StateMachine.add('CON', sm_con,
            transitions={'exit':'FINISH','fail':'FAILED'})

        
        
                
        
        
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome=sm.execute()
    sis.stop()
    rospy.spin()
    

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rospy.signal_shutdown()
