#!/usr/bin/env python3
import rospy
import threading
import time
import random
from std_msgs.msg import String

from consoleHandler import *
from systemController import *

def send_telemetry(pub:rospy.Publisher, x, y, z, speed, wheel_angle):
    telemetry_msg = f'SPEED:{speed}|POSITION:{x},{y},{z}|WHEEL_POSITION:{wheel_angle}'
    pub.publish(telemetry_msg)
    print(telemetry_msg)

class TelemetryRepeat(threading.Thread):
    def __init__(self, interval, func, args=None, kwargs=None):
        threading.Thread.__init__(self, target=func, args=args, kwargs=kwargs)
        self.interval = interval # seconds between calls
        self.func = func         # function to call
        self.args = args         # optional positional argument(s) for call
        # self.kwargs = kwargs     # optional keyword argument(s) for call
        self.runable = True
        self.telemetry_topic = 'telemetry'
        self.telemetry_pub = rospy.Publisher(self.telemetry_topic, String, queue_size=10)

    def run(self):
        while self.runable:
            x = random.randint(1,100)
            y = random.randint(1,100)
            z = random.randint(1,100)
            speed = random.randint(0,20)
            wheel_angle = random.randint(-50,50)
            # self.func(*self.args)
            self.func(self.telemetry_pub, x,y,z,speed,wheel_angle)
            time.sleep(self.interval)
    
    def stop(self):
        self.runable = False
    pass


if __name__ == '__main__':  
    telemetry_thread = TelemetryRepeat(3, func=send_telemetry)
    print(telemetry_thread.args)

    system_controller = SystemController('controlSystem_RPi4')
    telemetry_thread.start()
    system_controller.run()

    pass
