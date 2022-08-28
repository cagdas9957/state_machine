#!/usr/bin/env python
#-*-coding: utf-8 -*-

import smach_ros
import rospy
import smach
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu,PointCloud2
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray
import os

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['success'])
        self.check=True

    def execute(self,ud):
        while self.check==None:
            rospy.sleep()
        if self.check==True:
            return 'success'

class SensorCheck(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['failed'])
        self.imu_time=None
        self.odo_time=None
        self.lid_time=None

        self.imuworking = True
        self.encoderworking = True
        self.lidarworking = True

        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/velodyne_points', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/odometry/wheel', Odometry, self.encoder_callback)

        
    
    def imu_callback(self,data):
        time=rospy.Time.now().to_sec()

        self.imu_time=time

        if (data.angular_velocity == 0.0000):
            self.imuworking = False

        else:
            self.imuworking = True
            
       
            
    
    def encoder_callback(self, data):
        time=rospy.Time.now().to_sec()

        self.odo_time=time

        if (data.pose.pose.position.x == 0.0000):
            self.encoderworking = False
        
        else:
            self.encoderworking = True
    
    def lidar_callback(self, data):
        time=rospy.Time.now().to_sec()

        self.lid_time=time

        if (data.is_dense == False):
            self.lidarworking = False
            
        else:
            self.lidarworking = True            
       
    
    def execute(self, ud):
        rate=rospy.Rate(10)
        while True:
            rate.sleep()
            time=rospy.Time.now().to_sec()
            # print('FARK:',time-self.lid_time,time-self.odo_time)
            # print('VALUES:',self.lid_time,self.imu_time,self.odo_time)
            if self.lid_time==None or self.odo_time==None or self.imu_time==None:
                rospy.loginfo('Data Gelmiyor')
                os.system("rosnode kill /machina")
                return 'failed' 

            if (time-self.lid_time>5) or (time-self.odo_time>5) or (time-self.imu_time>5):
                rospy.loginfo('Zaman Aşimi')
                os.system("rosnode kill /machina")
                return 'failed'

            if self.encoderworking == True and self.imuworking == True and self.lidarworking == True :
                # rospy.loginfo('SENSOR CHECKED')
                rate.sleep()
                # rospy.loginfo("All sensors are working.")
            else:
                rospy.loginfo("zort")
                # self.pub.publish(True)
                os.system("rosnode kill /machina")     
                return 'failed'

class Pos(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['check','exit'],
                            output_keys=['counter'])
        self.counter=0
        self.pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

        rospy.loginfo(self.counter)

    def execute(self,userdata):
        komut=input('Pos Vermek ister misin yakisikli?(y=1/n=any key):')
    
        self.counter+=1
        userdata.counter=self.counter
        print("KANTIR: ", self.counter)
        # if self.counter<=3:
        if komut==1:
            rospy.loginfo("enter x pos:")
            self.pos_x=int(input() or 2.0)
            rospy.loginfo("enter y pos:")
            self.pos_y=int(input() or 2.0)
            rospy.loginfo("enter w pos:")
            self.pos_w=int(input() or 0.0)

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
            rospy.loginfo('STATE COMPLETED')
            os.system("rosnode kill /machina")
            return 'exit'

class Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['pos'])
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

            if self.check==3 : #self.check simülasyonda 3 olmalı
                rate.sleep()
                rospy.loginfo('POS STATE')
                self.check=None
                return 'pos'
                        
        # elif self.check==3 :
        #     rospy.loginfo('MOVE COMPLETED')
        #     self.check=None
        #     return 'exit'
    
def main():

    
    rospy.init_node('machina')

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
                                    transitions={'failed':'FAILED'})

        sm_main=smach.StateMachine(outcomes=['FINISHED'])

        with sm_main:
            smach.StateMachine.add('POS',Pos(),
                transitions={'check':'CHECK','exit':'FINISHED'})
            smach.StateMachine.add('CHECK',Check(),
                transitions={'pos':'POS'})

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
