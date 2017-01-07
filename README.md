# Sabertooth-Twist-Message-Control
Control sabertooth 2x60 by subscribing to twist messages

Writen in python, this ros node subscribes to cmd_vel from the telop_twist_joystick package by default and calculates wheel velocities for differential steering.

This connects to a Sabertooth 2x60 motor driver with address 128 by default and communicates by using the packetized serial method with 7 bit communication
