#! /usr/bin/env python

import rospy
from geometry_msgs import PointStamped
from usr_fun.msg import refree
if __name__=="__main__":
    rospy.init_node("smach")
    rospy.loginfo("hello world")
    goal_pub = rospy.Publisher("/goal_point",PointStamped,queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        rate.sleep()
    pass