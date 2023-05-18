#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8

class CameraHandler:
    def __init__(self) -> None:
        self.obstacles_topic = 'camera_obstacles'
        self.__obstacles_sub = rospy.Subscriber(self.obstacles_topic, Int8, self.__obstacles_callback)
        
        self.ctrl_topic = 'camera_control' # Для команд управления подсист. техн. зрения
        self.__ctrl_pub = rospy.Publisher(self.ctrl_topic, String, queue_size=10)

        # Возможно, пойдет в класс TechnicalController
        self.__ctrl_feedback_topic = 'camera_control_feedback' # Для ответа на команды управления подсист. техн. зрения
        self.__ctrl_feedback_sub = rospy.Subscriber(self.__ctrl_feedback_topic, String, self.__ctrl_feedback_callback)

        self.__camera_status = False
        self.__resolution = 480
        self.__distance = 5.0
        #

        self.n_obstacles = 0
        
    def receive(self):
        # rospy.spin()
        pass

    def __obstacles_callback(self, msg:Int8, verbose=True):
        if verbose:
            print(f"[{self.obstacles_topic}] Received: {msg.data}")
            
        self.n_obstacles = msg.data

    def __ctrl_feedback_callback(self, msg:String, verbose=True):
        if verbose:
            print(f"[{self.__ctrl_feedback_topic}] Received: {msg.data}")
        
        msg_split = msg.data.split(':')
        
        if msg_split[0] == 'status':
            if msg_split[1] == 'on':
                self.__camera_status = True
            elif msg_split[1] == 'off':
                self.__camera_status = False

        if msg_split[0] == 'resolution':
            self.__resolution = int(msg_split[1])

        if msg_split[0] == 'distance':
            self.__distance = float(msg_split[1])
    
    def get_info(self, verbose=True) -> list:
        params_list = [self.__camera_status, self.__resolution, self.__distance]

        if verbose:
            print("[INFO] Current camera parameters [status, resolution, distance]: " , params_list)
        
        return params_list

    def __send_str(self, msg:String, verbose:bool=True):
        try:
            self.__ctrl_pub.publish(msg)

            if verbose:
                print(f"[{self.ctrl_topic}] Sent: '{msg}'")

        except Exception as e:
            rospy.logerr(f" [{self.ctrl_topic}] Failed to send '{msg}' .\n{e}")

    def update_status(self):
        '''status -> status:on(off) - текущее состояние камеры (ВКЛ. / ВЫКЛ.)'''
        self.__send_str('status')

    def update_resolution(self):
        '''resolution - x:int -> resolution:x (x=480,720,1080) - текущее разрешение камеры'''
        self.__send_str('resolution')

    def update_distance(self):
        '''distance - x:float -> distance:x (x=5.0) - расстояние, на котором распознаются препятствия'''
        self.__send_str('distance')
    
    def set_status(self, enable:bool):
        '''status.on(off) -> status:on(off) - ВКЛ. (ВЫКЛ.) камеру'''
        self.__send_str('status.' + ('on' if enable else 'off'))

    def set_resolution(self, resolution:int):
        '''resolution.x -> resolution:x - изменить разрешение камеры'''
        if resolution < 240:
            resolution = 240
        
        elif resolution > 1080:
            resolution = 1080
        
        self.__send_str('resolution.' + str(resolution))
    
    def set_distance(self, distance:float):
        '''distance.x -> distance:x - изменить расстояние распознования препятствий'''
        if distance < 0.3:
            distance = 0.3
        
        elif distance > 500:
            distance = 500

        self.__send_str('distance.' + str(distance))
