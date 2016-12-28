#!/usr/bin/python
# coding: utf-8

import rospy
import serial
import numpy as np
import tf.transformations
from geometry_msgs.msg import Twist

address = 128
WHEEL_BASE = 10
WHEEL_RADIUS = 2
#function to scale values in one range to values in another range -  proportional scaling
def pscale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def motor1speed(address,speed):
    port = serial.Serial('/dev/ttyUSB0')
    port.isOpen()
    command = 6
    checksum = (address + command + speed) & 127
    print("M1 Speed: " + str(speed))
    port.write("".join(chr(i) for i in [address, 14, 50, (address + 64) & 127]))
    port.write("".join(chr(i) for i in [address, command, speed, checksum]))

def motor2speed(address,speed):
    port = serial.Serial('/dev/ttyUSB0')
    port.isOpen()
    command = 7
    checksum = (address + command + speed) & 127
    print("M2 Speed: " + str(speed))
    port.write("".join(chr(i) for i in [address, 14, 50, (address + 64) & 127]))
    port.write("".join(chr(i) for i in [address, command, speed, checksum]))

def callback(msg):
    linear_velocity_arr = np.array([msg.linear.x, msg.linear.y, msg.linear.z])
    angular_velocity_arr = np.array([msg.angular.x, msg.angular.y, msg.angular.z])

    linear_velocity = np.linalg.norm(linear_velocity_arr)
    angular_velocity = np.linalg.norm(angular_velocity_arr)
    print("twist received")
    rospy.loginfo("Twist Message")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    v1 = (msg.linear.x - msg.angular.z * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
    v2 = (msg.linear.x + msg.angular.z * WHEEL_BASE / 2.0)/WHEEL_RADIUS;
    print(v1) #min is -2.8 to 2.8
    print(v2)
    #to properly scale these numbers you need
    # to map the values from (-2.8,2.8) to (0,127)
    scaledv1 = pscale(v1,-1.5,1.5,0,127)
    scaledv2 = pscale(v2,-1.5,1.5,0,127)
    motor1speed(address,int(round(scaledv1,2)))
    motor2speed(address,int(round(scaledv2,2)))

def listener():

    rospy.init_node('sabertooth_driver') #anonymous=true would allow us to run multiple of these motor interpreters at once

    rospy.Subscriber("cmd_vel", Twist, callback)

    rospy.spin() #keeps python from exiting until node is stopped

if __name__ == '__main__':
    listener()
