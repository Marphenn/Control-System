#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def test_ros_sender(topic:str, msg:String): # pub:rospy.Publisher, 
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
            rate.sleep()

            pub.publish("left")
            # rospy.loginfo(f"[SENT] {msg}")
            rate.sleep()

            pub.publish("left")
            # rospy.loginfo(f"[SENT] {msg}")
            rate.sleep()

            pub.publish("left")
            # rospy.loginfo(f"[SENT] {msg}")
            rate.sleep()

            pub.publish("left")
            # rospy.loginfo(f"[SENT] {msg}")
            rate.sleep()

            pub.publish("brake")
            # rospy.loginfo(f"[SENT] {msg}")
            rate.sleep()

            pub.publish("brake")
            # rospy.loginfo(f"[SENT] {msg}")
            rate.sleep()
            
            feedback_pub.publish("joy:off")
            rate.sleep()

            # msg_feedback = "joy:connected"
            # feedback_pub.publish(msg_feedback)
            # rospy.loginfo(f"[SENT] {msg_feedback}")
            # rate.sleep()

            # msg = "break"
            # pub.publish(msg)
            # rospy.loginfo(f"[SENT] {msg}")
            # rate.sleep()

                      
    except Exception as e:
        rospy.logwarn(e)

if __name__ == "__main__":
    global pub
    global feedback_pub

    pubTopic = 'console_joy_control'
    subTopic = 'console_control'
    feedback_topic = 'console_control_feedback'

    pub = rospy.Publisher(pubTopic, String, queue_size=10)
    feedback_pub = rospy.Publisher(feedback_topic, String, queue_size=10)
    sub = rospy.Subscriber(subTopic, String)
    rospy.init_node('console')

    rate = rospy.Rate(1)
    
    try:
        # msg = "gas"
        test_ros_sender(topic=feedback_topic, msg=None)
 
    except rospy.ROSInterruptException:
        pass