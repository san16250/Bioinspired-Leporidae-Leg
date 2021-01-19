# Bioinspired-Leporidae-Leg

This repository contains all the programming in C for the operation of a
bioinspired leporidae leg around a GUI (uVision5 using TivaWare), for the ARM
MCU TM4C123GH6PMI.

This code was created by Christian Sandoval - 16250 under the supervision
of MSc. Miguel Zea in Universidad del Valle de Guatemala.

This code was implemented under the Embedded C coding standard by Michael
Barr.

##Known issues:
The IMU MPU6050 tends to start working after the first reset when the program
was first uploaded.

While making continuous jumps using UART, the system tends to get stuck.
(Ask for more details about this problem).
