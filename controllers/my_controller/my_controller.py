"""mavic2ros controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Keyboard
from controller import Motor
import math


import os

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())
# Get and enable devices.

IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

GYROsensor = robot.getGyro("gyro")
GYROsensor.enable(timeStep)

KeyB = robot.getKeyboard()
KeyB.enable(timeStep)

front_left_motor = robot.getMotor("front left thruster")
front_right_motor = robot.getMotor("front right thruster")
rear_left_motor = robot.getMotor("rear left thruster")
rear_right_motor = robot.getMotor("rear right thruster")
front_left_motor.setPosition(float('inf'))
front_right_motor.setPosition(float('inf'))
rear_left_motor.setPosition(float('inf'))
rear_right_motor.setPosition(float('inf'))
front_left_motor.setVelocity(0.0)
front_right_motor.setVelocity(0.0)
rear_left_motor.setVelocity(0.0)
rear_right_motor.setVelocity(0.0)

FL_wheel = robot.getMotor("fl wheel")
FR_wheel = robot.getMotor("fr wheel")
RL_wheel = robot.getMotor("rl wheel")
RR_wheel = robot.getMotor("rr wheel")
FL_wheel.setPosition(float('inf'))
FR_wheel.setPosition(float('inf'))
RL_wheel.setPosition(float('inf'))
RR_wheel.setPosition(float('inf'))
FL_wheel.setVelocity(0.0)
FR_wheel.setVelocity(0.0)
RL_wheel.setVelocity(0.0)
RR_wheel.setVelocity(0.0)

fly_wheel = robot.getMotor("flywheel")
fly_wheel.setPosition(float('inf'))
fly_wheel.setVelocity(0.0)

k_roll_p = 80.0           # P constant of the roll PID.
k_pitch_p = 100.0         # P constant of the pitch PID.
k_roll_d = 100.0
k_pitch_d = 120.0
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
target_altitude = 5.0
k_vertical_thrust = 67.1 # with this thrust, the drone lifts.
k_vertical_offset = 0.1   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 10.0        # P constant of the vertical PID.
k_vertical_d = 13000

def CLAMP(value, low, high):
    if value < low:
        return low
    elif value > high:
        return high
    return value

robot.step(timeStep)
xpos, altitude , zpos = GPSsensor.getValues()
xpos_old=xpos
altitude_old=altitude
zpos_old=zpos
roll_vel_old=0
lock_on = False
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw() 
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, pitch_vel, bleh =GYROsensor.getValues()
    print(str(roll)+"\t"+str(pitch)+"\t"+str(heading))
    xSpeed=(xpos-xpos_old)/timeStep
    ySpeed=(altitude-altitude_old)/timeStep
    zSpeed=(zpos-zpos_old)/timeStep
    #print(str(xSpeed)+"\t"+str(ySpeed)+"\t"+str(zSpeed))
    xpos_old=xpos
    altitude_old=altitude
    zpos_old=zpos
    #  val = ds.getValue()
    roll_disturbance = 0.0
    pitch_disturbance = 0.0
    yaw_disturbance = 0.0
    left=0
    right=0
    front_left_motor_input = 0
    front_right_motor_input = 0
    rear_left_motor_input = 0
    rear_right_motor_input = 0
    spin_boy = 0
    key=KeyB.getKey()
    while (key>0):
        if (key==KeyB.UP):
            #pitch_disturbance = 2.0
            left = 10
            right= 10
        if (key==KeyB.DOWN):
            #pitch_disturbance = -2.0
            left = -10
            right= -10
        if (key==KeyB.LEFT):
            #roll_disturbance = 1.0
            left = 10
            right= -10
        if (key==KeyB.RIGHT):
            #roll_disturbance = -1.0
            left = -10
            right= 10
        if (key==KeyB.HOME):
            front_left_motor_input = 100
            front_right_motor_input = 100
            rear_left_motor_input = 100
            rear_right_motor_input = 100
        if (key==KeyB.END):
            spin_boy = 1000
        if (key==ord('W')):
            pitch_disturbance = 0.261799
        if (key==ord('A')):
            roll_disturbance = 0.261799
        if (key==ord('S')):
            pitch_disturbance = -0.261799
        if (key==ord('D')):
            roll_disturbance = -0.261799
        if (key==ord('Q')):
            yaw_disturbance = 1
        if (key==ord('E')):
            yaw_disturbance = -1
        if (key==ord('Z')):
            target_altitude = altitude+0.1
        if (key==ord('X')):
            target_altitude = altitude-0.1
        if (key==ord('L')):
            lock_on = True
        if (key==ord('R')):
            roll_disturbance = -3
        key=KeyB.getKey()
    yaw_disturbance
    # Process sensor data here.
    #print(str(roll)+"\t"+str(pitch)+"\t"+str(heading))
    #print(str(roll)+"\t"+str(roll_vel))
    if abs(roll_vel_old-roll_vel)>2:
        k_roll_d=0
    else:
        k_roll_d=100.0
    roll_input = k_roll_p * (roll_disturbance-roll) - k_roll_d*roll_vel
    roll_vel_old=roll_vel
    pitch_input = (k_pitch_p *(pitch_disturbance-pitch) - k_pitch_d*pitch_vel)
    yaw_input = yaw_disturbance;
    
    
    vertical_input = k_vertical_p *CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)-k_vertical_d*ySpeed;
    if roll>math.pi/2 or roll<-math.pi/2:
        vertical_input=-vertical_input
        k_vertical_thrust=-67.1
    else:
        k_vertical_thrust=67.1
    #vertical_input = 0# k_vertical_p * pow(clamped_difference_altitude, 3.0);
    #0.2635 #0.266  #0.2635 #0.266  #roll distance
    #0.3582 #0.3582 #0.3346 #0.3346 #pitch distance
    #print(str(k_vertical_thrust)+"\t"+str(vertical_input)+"\t"+str(roll_input)+"\t"+str(pitch_input))
    if (lock_on==False):
        front_left_motor_input = k_vertical_thrust + vertical_input  + roll_input + pitch_input + yaw_input
        front_right_motor_input=(k_vertical_thrust + vertical_input) - roll_input + pitch_input - yaw_input
        rear_left_motor_input  =(k_vertical_thrust + vertical_input) + roll_input - pitch_input - yaw_input
        rear_right_motor_input =(k_vertical_thrust + vertical_input) - roll_input - pitch_input + yaw_input
    else:
        front_left_motor_input =-100
        front_right_motor_input=-100
        rear_left_motor_input  =-100
        rear_right_motor_input =-100
        lock_on= False
    #print(str(front_left_motor_input)+"\t"+str(front_right_motor_input)+"\t"+str(rear_left_motor_input)+"\t"+str(rear_right_motor_input))
    front_left_motor.setVelocity(CLAMP(-front_left_motor_input,-200,200))#positive is up  #0.44467908653
    front_right_motor.setVelocity(CLAMP(front_right_motor_input,-200,200))#negative is up #0.44616503673
    rear_left_motor.setVelocity(CLAMP(rear_left_motor_input,-200,200))#negative is up     #0.42589835641
    rear_right_motor.setVelocity(CLAMP(-rear_right_motor_input,-200,200))#positive is up  #0.42744959936
    fly_wheel.setVelocity(spin_boy)
    FL_wheel.setVelocity(left)
    FR_wheel.setVelocity(right)
    RL_wheel.setVelocity(left)
    RR_wheel.setVelocity(right)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.