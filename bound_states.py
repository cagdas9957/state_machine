#!/usr/bin/env python3
#-*-coding: utf-8 -*-

import rospy 
import smach
import smach_ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['move','stop'],
            output_keys=['vel_out'])
        
        rospy.Subscriber('/check_data',Bool,self.assign)
        self.check=True
        
    def assign(self,lidar_data):
        self.check=lidar_data
          
    def execute(self,userdata):
        
        if self.check ==True:
            userdata.vel_out=1.0
            rospy.loginfo('MOVE STATE')
            return 'move'
        else:
            rospy.loginfo('STOP STATE')
            return 'stop'

class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['check','stop'],
                input_keys=['move_vel_in'])
    
    def execute(self,userdata):
        tw=Twist()
        tw.linear.x=userdata.move_vel_in
        try:
            pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
            pub.publish(tw)
            rospy.loginfo('CHECK STATE')
            return 'check'
        except rospy.ROSInterruptException:
            print('Publish failed')
            rospy.loginfo('STOP STATE')
            return 'stop'

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['exit'])
    
    def execute(self,userdata):
        tw=Twist()
        pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        pub.publish(tw)
        rospy.loginfo('PROCESS TERMİNATED')
        return 'exit'

def main():

    
    rospy.init_node('bound_states',anonymous=True)

    sm=smach.StateMachine(outcomes=['finish'])
    


    with sm:
        smach.StateMachine.add('CHECK',Check(),
                transitions={'move':'MOVE','stop':'STOP'},
                remapping={'vel_out':'sm_data'})
        
        smach.StateMachine.add('MOVE',Move(),
                transitions={'check':'CHECK','stop':'STOP'},
                remapping={'move_vel_in':'sm_data'})
                
        
        smach.StateMachine.add('STOP',Stop(),
                transitions={'exit':'finish'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome=sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()