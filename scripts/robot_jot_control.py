#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32


previous_angle = 0
#steering_angle
#previous_steering_angle
#
#brake
#previous_brake
#
#transmission
#previous_transmission
#
#clutch
#previous_clutch


def callback(data):
    #передача
    buttonX = data.buttons[0]
    buttonC = data.buttons[1]
    buttonS = data.buttons[2]
    buttonT = data.buttons[3]

    #тормоз
    buttonL1 = data.buttons[4]
    buttonR1 = data.buttons[5]
    
    #сцепление
    transmission_ON = data.buttons[7]
    transmission_OFF = data.buttons[6]

    #угол поворота
    angle = int(10 * data.axes[0])
    linear = int(100 * data.axes[1])

    #
    #передача
    #
    # передача - B
    if buttonX == 1:
#        pub.publish(0)
        pub.publish(0x02000002)

    # передача - N
    if buttonC == 1:
#        pub.publish(1)
        pub.publish(0x02000000)

    # передача - N
    if buttonS == 1:
        pub.publish(0x02000000)

    # передача - F
    if buttonT == 1:
#        pub.publish(3)
        pub.publish(0x02000001)

    #
    #тормоз
    #        
    if buttonL1 == 1:
#        pub.publish(5)
#        pub.publish(33554432)
        pub.publish(0x00000000)

    if buttonR1 == 1:
#        pub.publish(5)
#        pub.publish(33554432)
        pub.publish(0x00000001)

    #
    #сцепление
    #
    if transmission_ON == 1:
        pub.publish(0x01000001)

    if transmission_OFF == 1:
        pub.publish(0x01000000)



    #
    #угол поворота
    #
    #if abs(angle) > 1:
    #if previous_angle != angle:
    if linear > 1:
        #previous_angle = angle
        if angle >= 0:
            pub.publish(0x03000000 + angle)
        else:
            pub.publish(0x03000080 - angle)
        #previous_angle = angle


def start():
    global pub

    pub = rospy.Publisher('robot_joy_control', Int32, queue_size=10)

    rospy.Subscriber("joy", Joy, callback)

    rospy.init_node('robot_joy_control', anonymous=True)
    rospy.spin()


if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

