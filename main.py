#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
 
# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click

# Create your objects here.
ev3 = EV3Brick()
left = Motor (Port.B)
right = Motor(Port.C)
arm = Motor (Port.A)
robot = DriveBase(left, right, wheel_diameter=80, axle_track=120)
speed = 40
steering= 0
time = 500
 
light = ColorSensor (Port .S1)
ultra = UltrasonicSensor(Port.S2)
arm.run(450)
# Write your program here.
while light.color() != Color.RED:
    correction = (-25 + light.reflection ())*2.5
    robot. drive(speed, correction)
    if ultra.distance() < 50:
        robot. turn(bee)
    if 90 > light.reflection() > 60:
        robot. stop()
    if light.color() == Color.GREEN:
        for i in range(18):
            robot.turn(10)
            if light.color() == Color.BLACK:
                for i in range(i):
                    robot.turn(-10)
            
                    
                    

        
        