
# These lines tell the robot which pre-built code it is going to use
#libs??

from pybricks import ev3brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
# Initialize the EV3 Brick.
ColourPort = Port.A
LeftMotorPort = Port.B
RightMotorPort = Port.C
ArmPort = Port()
UltrasonicPort = Port()
#class operations inherits ev3brick
class operations(ev3brick):
    def __init__(self):
        self.ev3 = super.ev3brick()
        self.motorLeft = Motor.port(LeftMotorPort)
        self.motorRight = Motor.port(RightMotorPort)
    def move(self, moveDict):
        self.motorRight.run_time(moveDict['Speed'])
        self.motorLeft.run_time(moveDict['Speed'])
        self.motorLeft.stop()
        self.motorRight.stop()
    def turn(self, turnDict):
        if turnDict["Angle"] >= 0:
            self.motorLeft.turn(turnDict['Angle'])
        elif turnDict["Angle"] < 0:
            self.motorLeft.turn(turnDict['Angle'])
    def colourDetect(self):
        return ColorSensor(ColourPort).color()
    def moveArm(self):
        pass
    def distanseDetect(self):
        distance = UltrasonicSensor(UltrasonicPort).distance()
        return distance
# Initialize the motors.


# Initialize the drive base.


# Drives at speed (mm/sec) while steering (degrees/sec) 
# robot.drive(speed, steering)
# until program ends or you give another command


#Loops a certain number of times
# for i in range(4):
# 	# Drives at speed (mm/sec) while steering (degrees/sec) 
# 	# for time milliseconds	
# 	drive_time(speed, steering, time) 
# 	counter = counter + 1

# Initialize the Ultrasonic Sensor.
#initialize the operations object
device = operations()
#Loops forever
while True:
        # while device.distanseDetect() > 100:
    while device.colourDetect() == Color.BLACK:
        moveDict = {
            "Speed" : 360,
            "Degrees" : 0
        }
        device.move(moveDict)
        moveDict = None
    while device.colourDetect != Color.BLACK:
        turnDict = {
                "Angle": "5"
            }
        turnedAngle = 0
        turnedAngle+=5
        device.turn(turnDict)
        if turnedAngle >= 90:
            turnDict = {
                "Angle" : "-90"
            }
            device.turn(turnDict)
            turnedAngle = 0
            turnDict = {
                "Angle" : "-5"
            }
    