#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

class CompassHandler:
    def __init__(self) -> None:
        self.x:float = 0.0
        self.y:float = 0.0
        self.z:float = None
        self.accel:float = 0.0
        self.speed:float = 0.0
        self.status:bool = True
        pass
    
    def get_data(self) -> str:
        msg = 'x:' + str(self.x) + 'y:' + str(self.y) + 'speed:' + str(self.speed)
        return msg

    pass