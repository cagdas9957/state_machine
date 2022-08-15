#!/usr/bin/env python3
#!-*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

def movebase_client(pub):
    
    goal = PoseStamped()
    goal.header.stamp=rospy.Time.now()
    goal.header.frame_id = "odom"
    goal.pose.position.x = 0.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0
    

    rate=rospy.Rate(2)
    rate.sleep()

    pub.publish(goal)
    # rate=rospy.Rate()
    # wait = client.wait_for_result()
    # if not wait:
    #     rospy.logerr("Action server not available!")
    #     rospy.signal_shutdown("Action server not available!")
    # else:
    #     return client.get_result()

if __name__ == '__main__':
    rospy.init_node('movebase_client')
    pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=10)

    movebase_client(pub)
        # if result:
        #     rospy.loginfo("Goal execution done!")
    