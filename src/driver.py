#!/usr/bin/python
# coding: utf-8

import rospy
import serial
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist
#reverse: address 128 motor 2 and address 129 motor 2
address1 = 129
address2 = 128
#WHEEL_BASE = 37
#WHEEL_RADIUS = 15
WHEEL_BASE = .61
WHEEL_RADIUS = .53
#function to scale values in one range to values in another range -  proportional scaling
def pscale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def motor1speed(address,speed):
    port = serial.Serial('/dev/ttyUSB0')
    if(speed > 127):
        speed = 127
    if(speed < 0):
        speed = 0
    port.isOpen()
    command1 = 6
    command2 = 7
    checksum1 = (address + command1 + speed) & 127
    checksum2 = (address + command2 + pscale(speed,0,127,127,0)) & 127
    #print("Left Speed: " + str(speed))
    port.write("".join(chr(i) for i in [address, 14, 10, (address + 24) & 127]))
    port.write("".join(chr(i) for i in [address, command1, speed, checksum1]))
    port.write("".join(chr(i) for i in [address, command2, pscale(speed,0,127,127,0), checksum2]))

def motor2speed(address,speed):
    port = serial.Serial('/dev/ttyUSB0')
    if(speed > 127):
        speed = 127
    if(speed < 0):
        speed = 0
    port.isOpen()
    command1 = 6
    command2 = 7
    checksum1 = (address + command1 + speed) & 127
    checksum2 = (address + command2 + pscale(speed,0,127,127,0)) & 127
   # print("Right Speed: " + str(speed))
    port.write("".join(chr(i) for i in [address, 14, 10, (address + 24) & 127]))
    port.write("".join(chr(i) for i in [address, command1, speed, checksum1]))
    port.write("".join(chr(i) for i in [address, command2, pscale(speed,0,127,127,0), checksum2]))

def callback(msg):
    print("twist received")
    rospy.loginfo("Twist Message")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    S = 3
    v1 = (msg.linear.x - msg.angular.z * S*WHEEL_BASE / 2.0)/WHEEL_RADIUS;
    v2 = (msg.linear.x + msg.angular.z * S*WHEEL_BASE / 2.0)/WHEEL_RADIUS;
    print(v1) #min is -2.8 to 2.8
    print(v2)
    #to properly scale these numbers you need
    # to map the values from (-2.8,2.8) to (0,127)
    scaledv1 = pscale(v1,-1.5,1.5,0,127)
    scaledv2 = pscale(v2,-1.5,1.5,0,127)
    print("Right Speed: " + str(scaledv1))
    print("Left Speed: " + str(scaledv2))
    motor1speed(address1,int(round(scaledv1,2)))
    motor2speed(address2,int(round(scaledv2,2)))

def listener():

    rospy.init_node('sabertooth_driver') #anonymous=true would allow us to run multiple of these motor interpreters at once

    rospy.Subscriber("zenith/cmd_vel", Twist, callback,queue_size = 2)
    #rospy.Subscriber("cmd_vel",Twist,callback,queue_size = 2)
    rospy.spin() #keeps python from exiting until node is stopped

if __name__ == '__main__':
    listener()
