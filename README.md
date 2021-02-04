https://hackaday.io/project/177313-smarter-motor-driver-smd-mod

# SMDMod
 Smarter Motor Driver (SMD) MOD

This projet is a tiny control board for DC motor, up to 3A. It provides a smart bus interface (TTL level, Up to 1Mbps) and configurable close-loop regulation for robotic applications. Any DC motor with or without reducer may be used and driven by this board (6-pin interface : 2 for DC motor, 4 for quadrature encoder A/B). The control board may be directly powered by a Lipo battery (up to 6s). 

Commercial off-the-shelf smart 3-pin cables, hubs, accessories and physical interface adapter boards (e.g. TTLinker) may be used to connect this module to the host controller (Arduino board, Arm-based board, RPi, and Jetson). User can drive up to 254 motors or servo, with the same bus.

Velocity and torque of each motor are controlable. Present velocity and torque feedback are avaible. Communication protocol is developed in Python and C/C++ (Arduino). Ping, read, write, sync write, reboot, factory reset commands are supported. One command takes about 1ms to be processed with a return status.

# SMSMod
 Smarter Micro Servo (SMS) MOD
https://hackaday.io/project/176678-smarter-micro-servo-sms-mod

SMS and SMB share a same bus interface/protocol. One host can drive several DC motor using SMD, and several micro/mini servo using SMS !

