#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

class EncoderHandler:
    def __init__(self) -> None:
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.rotation_speed = 0.0
        self.status = True
        pass

    def get_data(self) -> str:
        msg = 'left:' + str(self.left_angle) + 'right:' + str(self.right_angle)
        return msg

    pass