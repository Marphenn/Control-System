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

        # Возможно, пойдет в класс TechnicalController
        self.__ctrl_feedback_topic = 'pilot_system_control_feedback' # Для ответа на команды управления подсист. пилотир-я
        self.__ctrl_feedback_sub = rospy.Subscriber(self.__ctrl_feedback_topic, String, self.__ctrl_feedback_callback)
        #

        self.motors_cmds_list = [] # список команд упр-я моторами

    def receive(self):
        # rospy.spin()
        pass
    
    def send_motors(self, msg:Int32):
        ''' Send motor commands via ROS. '''
        
        try:
            self.__motors_pub.publish(msg)
            rospy.loginfo(f"[{self.motors_topic}] Отправлено (Sent): '{msg:#x}'")

        except Exception as e:
            rospy.logerr(f" [{self.motors_topic}] Ошибка отправки сообщения (Failed to send) '{msg:#x}' .\n{e}")

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
            print(f"[{self.ctrl_topic}] Отправлено (Sent): '{msg}'")

        except Exception as e:
            rospy.logerr(f" [{self.ctrl_topic}] Ошибка отправки сообщения (Failed to send) '{msg}' .\n{e}")
  
    def __ctrl_feedback_callback(self, msg:String):
        rospy.loginfo(f"[{self.__ctrl_feedback_topic}] Получено (Received): {msg.data}")

        pass