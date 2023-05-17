#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

class ToArduinoSender:
    def __init__(self) -> None:
        self.motor_topic = 'motors_control' # Предыдущая версия: 'robot_joy_control'
        self.__motors_pub = rospy.Publisher(self.motor_topic, Int32, queue_size=10)

        self.ctrl_topic = 'pilot_system_control' # Для команд управления подсист. пилотир-я
        self.__ctrl_pub = rospy.Publisher(self.ctrl_topic, String, queue_size=10)

        self.__ctrl_feedback_topic = 'pilot_system_control_feedback' # Для ответа на команды управления подсист. пилотир-я
        self.__ctrl_feedback_sub = rospy.Subscriber(self.__ctrl_feedback_topic, String, self.__ctrl_feedback_callback)
        pass

    def receive(self):
        # rospy.spin()
        pass
    
    def send(self, topic:str, msg:Int32):
        ''' Send motor commands and control commands via ROS.
            Control commands:
            steering_wheel_status - OK / ERROR
            driving_gear_status - OK / ERROR
            autopilot_status - ON / OFF
            autopilot.on - ВКЛ. автопилот
            autopilot.off - ВЫКЛ. автопилот
        '''
        try:
            if topic == self.motor_topic:
                self.__motors_pub.publish(msg)
            
            elif topic == self.ctrl_topic:
                self.__ctrl_pub.publish(msg)

            print(f"[{topic}] Sent: '{msg}'")

        except Exception as e:
            rospy.logerr(f" [{topic}] Failed to send '{msg}' .\n{e}")
  
    def __ctrl_feedback_callback(self, msg:String):
        print(f"[{self.__ctrl_feedback_topic}] Received: {msg.data}")

        pass