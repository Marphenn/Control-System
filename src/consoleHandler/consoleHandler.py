#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32


class ConsoleHandler:
	def __init__(self):
		# (!) Изменить на joy_msgs_list: в этом списке будут команды дист. упр-я, принятые с ВПУ
		self.joy_msgs_list = [] 
		#
		
		self.__joy_topic = 'console_joy_control'
		self.__joy_sub = rospy.Subscriber(self.__joy_topic, String, self.__joy_callback)

		self.__ctrl_topic = 'console_control'
		self.__ctrl_pub = rospy.Publisher(self.__ctrl_topic, String, queue_size=10)
		
		# Возможно, пойдет в класс TechnicalController
		self.__ctrl_feedback_topic = 'console_control_feedback'
		self.__ctrl_feedback_sub = rospy.Subscriber(self.__ctrl_feedback_topic, String, self.__ctrl_feedback_callback)
		#

		self.__telemetry_topic = 'console_telemetry'
		self.__telemetry_pub = rospy.Publisher(self.__telemetry_topic, String, queue_size=10)

		self.__remote_ctrl_is_ON = False
		self.__joy_is_connected = False
				
	@property
	def remote_ctrl_is_ON(self):
		return self.__remote_ctrl_is_ON
	
	# @property
	# def console_sub(self):
	# 	return self.__joy_sub
	
	def receive(self):
		# rospy.spin()
		pass

	def send_control(self, msg:str):
		''' 
		remote_ctrl_status - ON / OFF -> remote_ctrl_status:on(off)
		remote_ctrl.on - ВКЛ. дист. упр-е
		remote_ctrl.off - ВЫКЛ. дист. упр-е
		joy_status - (not) connected -> joy:(dis)connected
		'''	
		self.__ctrl_pub.publish(msg)
		print(f"[{self.__ctrl_topic}] Отправлено (Sent): {msg}")
		pass

	def send_telemetry(self, position:list, speed:int, wheel_rotation:float):
		'''
		position: [x,y] - координаты трактора
		speed: 25 - скорость трактора
		wheel_rotation: [-1 ... +1] - угол поворота руля
		'''
		
		if wheel_rotation < -1:
			wheel_rotation = -1

		elif wheel_rotation > 1:
			wheel_rotation = 1

		msg = "position:" + str(position[0]) + "," + str(position[1]) \
			+ ";speed:" + str(speed) \
			+ ";wheel_rotation:" + str(wheel_rotation)
		self.__telemetry_pub.publish(msg)
		print(f"[{self.__telemetry_topic}] Отправлено (Sent): {msg}")
		pass

	def joy_to_motors_cmds(self) -> list:
		motors_cmds_list = []
		
		for msg in self.joy_msgs_list:
			# Перекодировка команд для дальнейшей отправки на Arduino. (Этот список будет в классе pilotSystemHandler)
			if msg == "gas":		
				# Сообщение для отправки на Arduino
				motors_cmds_list.append(0x02000001)

			elif msg == "brake":
				motors_cmds_list.append(0x00000001)

			elif msg == "left":
				motors_cmds_list.append(0x03000000)

			elif msg == "right":
				motors_cmds_list.append(0x03000001)
			
			else:
				rospy.logerr('[ERROR] Неверный формат команды дистанционного управления.\n \
		 		Список допустимых команд:\n \
		 		\t-gas - движение вперед\n \
		 		\t-brake - тормоз\n \
		 		\t-left - поворот колес влево\n \
		 		\t-right - поворот колес вправо')
				self.joy_msgs_list.remove(msg)

		# print(f'Список команд для отправки в подсистему пилотирования:{motors_cmds_list}')
		self.joy_msgs_list.clear() # Очистить список команд с геймпада
		
		return motors_cmds_list
	
	def __joy_callback(self, msg:String):
		rospy.loginfo(f"[{self.__joy_topic}] Получено (Received): {msg.data}")
		
		# Добавление полученной команды в список команд управления с геймпада
		self.joy_msgs_list.append(msg.data)

	def __ctrl_feedback_callback(self, msg:String):
		rospy.loginfo(f"[{self.__ctrl_feedback_topic}] Получено (Received): {msg.data}")

		if msg.data.startswith("joy:"):
			if msg.data == "joy:connected":
				self.__joy_is_connected = True

			elif msg.data == "joy:disconnected":
				self.__joy_is_connected = False

			else:
				rospy.logerr(f"[ERROR] '{msg.data}' - Неверное значение параметра.")
				return

		elif msg.data.startswith("remote_ctrl:"):
			if msg.data == "remote_ctrl:on":
				self.__remote_ctrl_is_ON = True

			elif msg.data == "remote_ctrl:off":
				self.__remote_ctrl_is_ON = False
			
			else:
				rospy.logerr(f"[ERROR] '{msg.data}' - Неверное значение параметра.")
				return

		else:
			rospy.logerr(f"[ERROR] '{msg.data}' - Неверный формат.")
			return
		
		# if msg.data == "joy:connected":
		# 	self.__joy_is_connected = True

		# elif msg.data == "joy:disconnected":
		# 	self.__joy_is_connected = False

		# elif msg.data == "remote_ctrl:on":
		# 	self.__remote_ctrl_is_ON = True

		# elif msg.data == "remote_ctrl:off":
		# 	self.__remote_ctrl_is_ON = False
		
		# else:
		# 	print(f"[ERROR] '{msg.data}' - Неверный формат команды.")
		# 	return

		print(f"Геймпад подключен (Joy is connected): {self.__joy_is_connected}")
		print(f"Удаленное управление включено (Remote control is ON): {self.__remote_ctrl_is_ON}")

		pass
		