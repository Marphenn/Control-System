#!/usr/bin/env python3
import rospy

from consoleHandler import *
from pilotSystemHandler import *
from cameraHandler import *

class SystemController:
    def __init__(self):
        rospy.init_node('controlSystem_RPi4')
        rospy.loginfo("Started node!")
    
    def run(self):
        console_handler = ConsoleHandler()
        arduino_sender = PilotSystemHandler()
        camera_handler = CameraHandler()

        while not rospy.is_shutdown():
            console_handler.receive()

            for msg in console_handler.motor_msgs_list:
                arduino_sender.send_motors(arduino_sender.motors_topic, msg)
            console_handler.motor_msgs_list.clear()

            camera_handler.get_info()

            # Testing telemetry sending
            test_position = [24, 77]
            test_speed = 20
            test_wheel_rot = 0.65

            console_handler.send_telemetry(test_position, test_speed, test_wheel_rot)
            rate = rospy.Rate(1)
            rate.sleep()


            # arduino_sender.send(arduino_sender.ctrl_topic, "driving_gear_status")

            pass
