#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    global pub
    global feedback_pub

    subTopic = 'motors_control'
    sub_control_topic = 'pilot_system_control'
    feedback_topic = 'pilot_system_control_feedback'

    camera_ctrl_topic = 'camera_control_feedback'

    pub = rospy.Publisher(feedback_topic, String, queue_size=10)
    sub = rospy.Subscriber(subTopic, String)
    sub_control = rospy.Subscriber(sub_control_topic, String)
    rospy.init_node('pilot_system')

    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        pub.publish('Test message')
        pass