#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    global pub
    global feedback_pub

    subTopic = 'camera_control'
    camera_ctrl_topic = 'camera_control_feedback'

    sub = rospy.Subscriber(subTopic, String)
    sub_control = rospy.Subscriber(camera_ctrl_topic, String)
    rospy.init_node('camera')

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        pass