#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32


class ConsoleHandler:
	def __init__(self):
		self.motor_msgs_list = []

		self.__joy_topic = 'console_joy_control'
		self.__console_sub = rospy.Subscriber(self.__joy_topic, String, self.__joy_callback)

		self.__ctrl_topic = 'console_control'
		self.__console_pub = rospy.Publisher(self.__ctrl_topic, String, queue_size=10)
		
		self.__ctrl_feedback_topic = 'console_control_feedback'
		self.__ctrl_feedback_sub = rospy.Subscriber(self.__ctrl_feedback_topic, String, self.__ctrl_feedback_callback)

		self.__remote_ctrl_is_ON = False
		self.__joy_is_connected = False
			
	@property
	def console_sub(self):
		return self.__console_sub
	
	def receive(self):
		# rospy.spin()
		pass

	def send(self, msg:str):
		''' 
			remote_ctrl_status - ON / OFF
			remote_ctrl.on - ВКЛ. дист. упр-е
			remote_ctrl.off - ВЫКЛ. дист. упр-е
			joy_status - (not) connected
		'''	
		self.__console_pub.publish(msg)
		print(f"[{self.__ctrl_topic}] Sent: {msg}")
		pass
	
	def __joy_callback(self, msg:String):
		print(f"[{self.__joy_topic}] Received: {msg.data}")
		
		# self.__handle_msg(msg.data)
		# Перекодировка команд для дальнейшей отправки на Arduino
		if msg == "gas":		
			# Сообщение для отправки на Arduino
			self.motor_msgs_list.append(0x02000001)

		elif msg == "brake":
			self.motor_msgs_list.append(0x00000001)

		elif msg == "left":
			self.motor_msgs_list.append(0x03000000)

		elif msg == "right":
			self.motor_msgs_list.append(0x03000001)


	def __ctrl_feedback_callback(self, msg:String):
		print(f"[{self.__ctrl_feedback_topic}] Received: {msg.data}")

		if msg.data == "joy:connected":
			self.__joy_is_connected = True

		elif msg.data == "joy:disconnected":
			self.__joy_is_connected = False

		elif msg.data == "remote_ctrl:on":
			self.__remote_ctrl_is_ON = True

		elif msg.data == "remote_ctrl:off":
			self.__remote_ctrl_is_ON = False

		print(f"Joy is connected: {self.__joy_is_connected}")
		print(f"Remote control is ON: {self.__remote_ctrl_is_ON}")

		pass
		