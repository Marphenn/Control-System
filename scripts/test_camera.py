#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    global pub
    global feedback_pub

    cameraTopic = 'tech_view'
    camera_ctrl_topic = 'tech_view_control'

    pub = rospy.Publisher(cameraTopic, String, queue_size=10)
    sub_control = rospy.Subscriber(camera_ctrl_topic, String)
    rospy.init_node('camera')

    rate = rospy.Rate(0.1)
    
    while not rospy.is_shutdown():
        msg = 'FACES_NUMBER:1'
        pub.publish(msg)
        rate.sleep()
        pass