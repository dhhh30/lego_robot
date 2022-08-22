
# These lines tell the robot which pre-built code it is going to use
#libs??
from pybricks import ev3brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
class sensors():
    def __init__():
        pass
    #Turning function
    def turn(turnDegrees, robot):
        robot.turn(turnDegrees)
    #Target detection function
    def distanceDetection(robot):
        distance = obstacle_sensor.distance()
	# Checks if an obstacle is within 100mm (10cm)
        if distance < 100:
		#Turns the robot 120 degrees
            sensors.turn("120", robot)
            
def init():
    ev3 = ev3brick()
    left_motor = Motor(Port.B)
    right_motor = Motor(Port.C)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
    
    deviceTuple = (ev3, left_motor, right_motor,robot)
    return deviceTuple
# Initialize the EV3 Brick.


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
obstacle_sensor = UltrasonicSensor(Port.S4)

robot_objs = init()
#Loops forever
while True:
	# Measures the distance between the ultrasonic sensor and an obstacle in millimetres
    sensors.distanceDetection(robot_objs[3])
