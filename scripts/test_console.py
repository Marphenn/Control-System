#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def ros_sender(msg:String): # pub:rospy.Publisher, 
    '''Send commands via ROS'''
    try:
        # pub.publish(msg)
        # rospy.loginfo(f"[SENT] {msg}")

        # For debugging
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            msg = "gas"
            pub.publish(msg)
            rospy.loginfo(f"[SENT] {msg}")

            msg_feedback = "resolution:720"
            feedback_pub.publish(msg_feedback)
            rospy.loginfo(f"[SENT] {msg_feedback}")

            rate.sleep()

            msg = "brake"
            pub.publish(msg)
            rospy.loginfo(f"[SENT] {msg}")

            msg_feedback = "resolution:240"
            feedback_pub.publish(msg_feedback)
            rospy.loginfo(f"[SENT] {msg_feedback}")

            rate.sleep()

            # msg = "left"
            # pub.publish(msg)
            # rospy.loginfo(f"[SENT] {msg}")
            # rate.sleep()

            # msg = "right"
            # pub.publish(msg)
            # rospy.loginfo(f"[SENT] {msg}")
            # rate.sleep()
            
    except Exception as e:
        rospy.logwarn(e)

if __name__ == "__main__":
    global pub
    global feedback_pub

    pubTopic = 'console_joy_control'
    pub = rospy.Publisher(pubTopic, String, queue_size=10)

    feedback_topic = 'console_control_feedback'
    camera_ctrl_topic = 'camera_control_feedback'
    feedback_pub = rospy.Publisher(camera_ctrl_topic, String, queue_size=10)
    rospy.init_node('console_test')
    try:
        # msg = "gas"
        ros_sender(None)
    except rospy.ROSInterruptException:
        pass