#!/usr/bin/env python3
import rospy

from consoleHandler import *
from pilotSystemHandler import *
from cameraHandler import *

class SystemController:
    def __init__(self, node_name:str):
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.loginfo("Started node!")
    
    def run(self):
        console_handler = ConsoleHandler()
        pilot_system_handler = PilotSystemHandler()
        camera_handler = CameraHandler()

        # For real use
        while not rospy.is_shutdown():
            ''' +. Послать сигнал инициализации на ВПУ, ПП, ПТЗ. Инициализация подсистемы:
                   запущена нода; созданы publisher-ы / subscriber-ы; флаг status(isReady) -> True.
                   Подсистема готова отправлять и принимать сообщения. 
                +. Опросить состояние ВПУ, ПП, ПТЗ.

                -- К этому моменту все подсистемы запущены и готовы к работе --
                1. Обработка ВПУ:
                    *1.1. Принять команды дист. упр-я с геймпада (автом.).
                    - (!) Изменить обработку (теперь будет два списка). *1.2. Обработать команды дист. упр-я (автом. joy_callback).
                          Обработка: создание соотв. команды для ПП и добавление её в список команд упр-я моторами для ПП.
                    +. Принять маршрутные команды.
                    +. Обработать маршрутные команды (добавить в список маршр. команд для ПП ?).
                2. Обработка ПП:
                    2.1. Отправить готовые команды из списка команд упр-я моторами для ПП.
                    +. Отправить маршрутные команды для ПП.
                    +. Принять отчёт о прохождении текущего маршрута 
                       (пройденный % , кол-во пройденных / оставшихся точек ?). 
                3. Обработка ПТЗ:
                    *3.1. Принять данные о препятствиях с камеры (автом.)
                    - 3.2. Обработка данных о препятствиях (если они есть).
                         Обработка: если есть препятствие, добавить команду "Тормоз" в список команд упр-я моторами (!).
                         Прекратить принимать команды дист. упр-я с ВПУ (либо у себя это сделать через флаг, 
                         либо отпр. команду упр-я на ВПУ "remote_ctrl:off")
            '''
            console_handler.receive() # *1.1 (автом.) 
            # console_handler.__joy_callback(msg) # *1.2 (автом.) 

            # 2.1
            for msg in console_handler.motor_msgs_list:
                pilot_system_handler.send_motors(msg)
            console_handler.motor_msgs_list.clear() # удалить отправленные команды из списка команд для ПП
            #

            camera_handler.receive() # *3.1 (автом.)


        # For debugging and testing
        while not rospy.is_shutdown():
            # Test receiving joy msgs from console and sending motor msgs to pilot system
            console_handler.receive()

            for msg in console_handler.motor_msgs_list:
                pilot_system_handler.send_motors(msg)
            console_handler.motor_msgs_list.clear()

            # Test receiving from camera
            camera_handler.get_info()

            # Test telemetry sending
            test_position = [24, 77]
            test_speed = 20
            test_wheel_rot = 0.65

            console_handler.send_telemetry(test_position, test_speed, test_wheel_rot)
            rate = rospy.Rate(1)
            rate.sleep()

            # arduino_sender.send(arduino_sender.ctrl_topic, "driving_gear_status")

    
    def shutdown(self, reason:str='[] System shutdown.'):
        rospy.signal_shutdown(reason)
