#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import sqrt

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# BRAIN.
ev3 = EV3Brick()

# Driving Motor.
left_wheel = Motor(Port.A)
right_wheel = Motor(Port.B)

# Claw motor.
claw_motor = Motor(Port.C)
# 9 24 92

robot = DriveBase(left_wheel, right_wheel, 36, 133)
# Colour Sensors facing ground.
left_light_sensor = ColorSensor(Port.S2)
right_light_sensor = ColorSensor(Port.S3)
# Front Colour Sensor.
front_colour_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S4)


# left colour sensor
# White: r: 39 g: 46 b: 98
# Black: r: 0 g: 3 b: 2

# right colour sensor
# White: r: 34 g: 36 b: 100
# Black: r: 2 g: 3 b: 5

# Declaring objects.
left_min_red = 1
left_max_red = 39

left_min_green = 4
left_max_green = 46

left_min_blue = 2
left_max_blue = 98

right_min_red = 2
right_max_red = 34

right_min_green = 3
right_max_green = 36

right_min_blue = 7
right_max_blue = 100

left_red_range = left_max_red - left_min_red
left_green_range = left_max_green - left_min_green
left_blue_range = left_max_blue - left_min_blue

right_red_range = right_max_red - right_min_red
right_green_range = right_max_green - right_min_green
right_blue_range = right_max_blue - right_min_blue

silver = False
green_square = False
block = False
obs = False
deposit_area_found = False
long_corner = True
# Starting beep.
ev3.speaker.beep()

# Functions.
def calibrate_values():
    left_val = left_light_sensor.rgb()
    
    left_red_cal = max(min((left_val[0] - left_min_red)/left_red_range,1),0)
    left_green_cal = max(min((left_val[1] - left_min_green)/left_green_range,1),0)
    left_blue_cal = max(min((left_val[2] - left_min_blue)/left_blue_range,1),0)


    right_val = right_light_sensor.rgb()

    right_red_cal = max(min((right_val[0] - right_min_red)/right_red_range,1),0)
    right_green_cal = max(min((right_val[1] - right_min_green)/right_green_range,1),0)
    right_blue_cal = max(min((right_val[2] - right_min_blue)/right_blue_range,1),0)
    
    return (left_red_cal, left_green_cal, left_blue_cal, right_red_cal, right_green_cal, right_blue_cal)

def print_values(side):
    if side == "left": 
        ev3.screen.print("l: " + str(left_light_sensor.rgb()))
    else:
        ev3.screen.print("r: " + str(right_light_sensor.rgb()))

# Lr: 0.3, Lg: 0.6, Lb: 0.3   
# Rr: 0.2 Rg: 0.6 Rb: 0.3    
def line_track(l_red, l_green, l_blue, r_red, r_green, r_blue):

    error = l_green - r_green
    speed = (0.8 - abs(error)) * 125
    rotation = error * 200
    if l_green <= 0.4 and r_green <= 0.4:
        print("double black")
        robot.stop()
        robot.straight(-30)
        
        val = calibrate_values()
        left_green = sqrt((0.3 - val[0]) ** 2 +  (0.6 - val[1]) ** 2 + (0.2 - val[2]) ** 2)
        right_green = sqrt((0.2 - val[3]) ** 2 + (0.6 - val[4]) ** 2 + (0.2 - val[5]) ** 2)
        
        print(left_green, right_green)

        if left_green <= 0.5:
            if right_green <= 0.2:
                print("turn 180")
                robot.turn(180)
                robot.drive(0, 30)
                while sum(calibrate_values()[:3]) > 1.5:
                    pass
            else:
                robot.straight(60)
                robot.turn(-95)
                robot.drive(0, -30)
                while sum(calibrate_values()[3:]) > 1.5:
                    pass
        else:
            if right_green <= 0.5:
                robot.straight(60)
                robot.turn(100)
                robot.drive(0, 30)
                while sum(calibrate_values()[:3]) > 1.5:
                    pass
            else:
                start = robot.distance()
                while robot.distance() - start < 70:
                    values = calibrate_values()
                    error = values[1] - values[4]
                    speed = (0.8 - abs(error)) * 140
                    rotation = error * 200
                    robot.drive(speed, rotation)
    else:
    #ev3.screen.print(speed, rotation)
        robot.drive(speed, rotation)
# 9 24 92
def detect_cube():
    global block, obs
    #print(front_colour_sensor.rgb())
    front_val = front_colour_sensor.rgb()
    if sum(front_val) > 10:
        robot.straight(20)
        updated_front_val = front_colour_sensor.rgb()
        if (updated_front_val[2]/sum(updated_front_val)) > .69:
            print("claw activated")
            block = True

        else:
            obs = True

def avoid():
    print(max(min(30, 50 - ultrasonic_sensor.distance()),-30))

def pick_cube():
    robot.straight(30)
    claw_motor.run_time(150, 2000)
    claw_motor.run_time(-150, 2000)
    robot.straight(-50)
    

def detect_evacuation_zone():
    if right_light_sensor.rgb()[0] >= 55 and right_light_sensor.rgb()[1] >= 58 and right_light_sensor.rgb()[2] >= 100 and left_light_sensor.rgb()[0] >= 77 and left_light_sensor.rgb()[1] >= 100 and left_light_sensor.rgb()[2] >= 100:
        print("silver")
        silver = True
        
    else:
        print("silver not detected")
        silver = False

def detect_ball():
    #silver ball: 15 15 40
    if front_colour_sensor.rgb()[0] > 0 and front_colour_sensor.rgb()[1] > 0 and front_colour_sensor.rgb()[2] > 0:
        before_dist = robot.distance()
        robot.drive(50, 0)
        claw_motor.run_time(100, 1700)
        claw_motor.stop()
        claw_motor.run_time(350, 1000)
        robot.stop()
        claw_motor.run_time(-130, 2300)
        robot.straight(before_dist - robot.distance())
       

        

def evacuation_zone():
    global distance_from_wall
    global deposit_area_found
    global long_corner
    robot.reset()
    if ultrasonic_sensor.distance() > 10:
        pass
    robot.straight(140)
    print("start")
    
    distance_from_wall = ultrasonic_sensor.distance()
    if long_corner == True:
        while robot.distance() < 680:
            detect_ball()
            robot.drive(50, max(min(10, distance_from_wall - ultrasonic_sensor.distance()),-10))
        long_corner = False
    else:
        while robot.distance() < 390:
            detect_ball()
            robot.drive(50, max(min(10, distance_from_wall - ultrasonic_sensor.distance()),-10))
        long_corner = True
    robot.stop()
    robot.turn(45)
    robot.straight(225)
    if deposit_area_found == False:
        if ultrasonic_sensor.distance() > 100:
            print("go back")
            robot.straight(-225)
            robot.turn(-45)
            robot.reset()
            while robot.distance() < 220:
                detect_ball()
                robot.drive(50, max(min(10, distance_from_wall - ultrasonic_sensor.distance()),-10))
            robot.stop()
            robot.turn(90)
            
        else:
            robot.turn(100)
            robot.straight(-90)
            robot.straight(100)
            robot.turn(-90)
            robot.straight(100)
            robot.turn(45)
            robot.reset()
            while robot.distance() < 180:
                detect_ball()
                robot.drive(50, max(min(10, distance_from_wall - ultrasonic_sensor.distance()),-10))
            robot.stop()
            print("deposit ball")
            deposit_area_found = True
    else:
        print("no depo")
        robot.turn(90)
        
    


while True:
    #detect_ball()
    evacuation_zone()
while False:
    detect_evacuation_zone()

    while silver == False:
    #left_blue_cal < left_max_blue or left_green_cal < left_max_green or left_red_cal < left_max_red or right_blue_cal < right_max_blue or right_green_cal < right_max_green or right_red_cal < right_max_red:
        values = calibrate_values()
        detect_cube()
        if block == False:
            
            if obs == False:
                line_track(values[0], values[1], values[2], values[3], values[4], values[5])

            else:
                robot.straight(-50)
                robot.stop()
                robot.turn(80)
                robot.drive(0, 30)
                while ultrasonic_sensor.distance() > 60:
                    pass
                while sum(calibrate_values()) > 3:
                    robot.drive(50, max(min(30, 50 - ultrasonic_sensor.distance()),-25)) 

                robot.straight(60)
                robot.turn(75)
                robot.drive(0, 30)
                while sum(calibrate_values()[:3]) > 1.5:
                    pass
                obs = False
        else:
            robot.stop()
            pick_cube()
            block = False
    else:
        detect_ball()
        
        
    