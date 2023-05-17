#!/usr/bin/env python3
import rospy

from consoleHandler import *
from senders import *

class SystemController:
    def __init__(self):
        rospy.init_node('controlSystem_RPi4')
        rospy.loginfo("Started node!")
    
    def run(self):
        console_handler = ConsoleHandler()
        arduino_sender = ToArduinoSender()

        while not rospy.is_shutdown():
            console_handler.receive()

            for msg in console_handler.motor_msgs_list:
                arduino_sender.send(arduino_sender.motor_topic, msg)
            console_handler.motor_msgs_list.clear()

            # arduino_sender.send(arduino_sender.ctrl_topic, "driving_gear_status")

            pass
