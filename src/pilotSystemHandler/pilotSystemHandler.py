#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

class PilotSystemHandler:
    def __init__(self) -> None:
        self.motors_topic = 'motors_control' # Предыдущая версия: 'robot_joy_control'
        self.__motors_pub = rospy.Publisher(self.motors_topic, Int32, queue_size=10)

        self.ctrl_topic = 'pilot_system_control' # Для команд управления подсист. пилотир-я
        self.__ctrl_pub = rospy.Publisher(self.ctrl_topic, String, queue_size=10)

        self.__ctrl_feedback_topic = 'pilot_system_control_feedback' # Для ответа на команды управления подсист. пилотир-я
        self.__ctrl_feedback_sub = rospy.Subscriber(self.__ctrl_feedback_topic, String, self.__ctrl_feedback_callback)
        pass

    def receive(self):
        # rospy.spin()
        pass
    
    def send_motors(self, msg:Int32):
        ''' Send motor commands via ROS. '''
        
        try:
            self.__motors_pub.publish(msg)
            print(f"[{self.motors_topic}] Sent: '{msg:#x}'")

        except Exception as e:
            rospy.logerr(f" [{self.motors_topic}] Failed to send '{msg:#x}' .\n{e}")

    def send_control(self, msg:String):
        ''' Send control commands via ROS. \n
            Control commands: \n
            steering_wheel_status - OK / ERROR \n
            driving_gear_status - OK / ERROR \n
            autopilot_status - ON / OFF \n
            autopilot.on - ВКЛ. автопилот \n
            autopilot.off - ВЫКЛ. автопилот \n
        '''
        
        try:
            self.__ctrl_pub.publish(msg)
            print(f"[{self.ctrl_topic}] Sent: '{msg}'")

        except Exception as e:
            rospy.logerr(f" [{self.ctrl_topic}] Failed to send '{msg}' .\n{e}")
  
    def __ctrl_feedback_callback(self, msg:String):
        print(f"[{self.__ctrl_feedback_topic}] Received: {msg.data}")

        pass