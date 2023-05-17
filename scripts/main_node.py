#!/usr/bin/env python3

import rospy

from consoleHandler import *
from systemController import *

if __name__ == '__main__':
    system_controller = SystemController()
    system_controller.run()
    pass