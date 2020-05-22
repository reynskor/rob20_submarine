"""controller_ver3 controller."""
# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import InertialUnit
from controller import Gyro
from controller import Keyboard
from controller import Motor
import math
import time
#ROS#import rospy
import os

#ROS#from geometry_msgs.msg import Vector3
#ROS#from std_msgs.msg import Int16
#ROS#from std_msgs.msg import Float32
#ROS#from std_msgs.msg import String
#ROS#from sensor_msgs.msg import Image

# create the Robot instance.
robot = Robot()

pitch_disturbance = 0
roll_disturbance = 0
yaw_disturbance = 0
def crying_orangutan(data):
    # pitch - data.data is either 1,0,-1
    global pitch_disturbance
    pitch_disturbance = 0.349066*data.data
    pass
    
def crying_polar_bears(data):
    global roll_disturbance
    roll_disturbance = 0.349066*data.data
    pass
    
def crying_animals_in_general(data):
    global yaw_disturbance
    yaw_disturbance = 5*data.data
    pass
    
def crying_mother_earth_in_general(data):
    global altitude
    global target_altitude
    if data.data==1:
        target_altitude = altitude+0.1
    elif data.data==-1:
        target_altitude = altitude+0.1
        
    pass
    
    
# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())
print("Time Step is: "+str(timeStep))
# Get and enable devices.
#ROS#rospy.init_node('python_submarine_controller', anonymous=True) # node is called 'python_webots_controller'
#ROS#rospy.loginfo("Loading Webots Controller")
#ROS#pub = rospy.Publisher('imu_values_topic', Vector3, queue_size=10)
#ROS#depth_pub = rospy.Publisher('depth_topic', Float32, queue_size=10)
#ROS#log_pub = rospy.Publisher('python_submarine_logger', String, queue_size=10)
#ROS#camera_pub = rospy.Publisher('python_submarine_camera_images', Image, queue_size=10)
#ROS#rearcamera_pub = rospy.Publisher('python_submarine_rear_camera_images', Image, queue_size=10)
#ROS#bleh_pub = rospy.Publisher("python_submarine_heading_speed",Float32,queue_size=10)
#ROS#speed_pub = rospy.Publisher('python_submarine_speeds', Vector3, queue_size=10)


#ROS#rospy.Subscriber("pitch_control_input", Int16, crying_orangutan)
#ROS#rospy.Subscriber("roll_control_input", Int16, crying_polar_bears)
#ROS#rospy.Subscriber("heading_control_input", Int16, crying_animals_in_general)
#ROS#rospy.Subscriber("altitude_control_input", Int16, crying_mother_earth_in_general)

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

camera = robot.getCamera("camera")
camera.enable(timeStep)

rearcamera = robot.getCamera("rearcamera")
rearcamera.enable(timeStep)

FL_wheel = robot.getMotor("front left wheel")
FR_wheel = robot.getMotor("front right wheel")
RL_wheel = robot.getMotor("rear left wheel")
RR_wheel = robot.getMotor("rear right wheel")
FL_wheel.setPosition(float('inf'))
FR_wheel.setPosition(float('inf'))
RL_wheel.setPosition(float('inf'))
RR_wheel.setPosition(float('inf'))
FL_wheel.setVelocity(0.0)
FR_wheel.setVelocity(0.0)
RL_wheel.setVelocity(0.0)
RR_wheel.setVelocity(0.0)

#fly_wheel = robot.getMotor("flywheel")
#fly_wheel.setPosition(float('inf'))
#fly_wheel.setVelocity(0.0)

k_roll_p = 600         # P constant of the roll PID.
k_pitch_p = 600       # P constant of the pitch PID.
k_yaw_p = 500
k_roll_d = 800
k_pitch_d = 800
k_yaw_d = 300
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
target_altitude = 9.0
k_vertical_thrust = 398.3476#187.28#592.2569#398.3476#327.3495#128.1189 # with this thrust, the drone lifts.
#k_vertical_offset = 0.1   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 300       # P constant of the vertical PID.
k_vertical_d = 1000

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
drone_mode = True
atache_mode = False
car_mode = False
altitude_bool=False
angle_dist = 0.785398
angle_lock = -2.0944
logger=False
#ROS#depth_msg = Float32()
pi = math.pi
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    roll, pitch, heading = IMUsensor.getRollPitchYaw() 
    xpos, altitude , zpos = GPSsensor.getValues()
    roll_vel, bleh, pitch_vel =GYROsensor.getValues()
    #print(str(roll_vel)+"\t"+str(pitch_vel))
    littleTimeStep = timeStep/1000.0
    xSpeed=(xpos-xpos_old)/littleTimeStep
    ySpeed=(altitude-altitude_old)/littleTimeStep
    zSpeed=(zpos-zpos_old)/littleTimeStep
    #print(str(xSpeed)+"\t"+str(ySpeed)+"\t"+str(zSpeed))
    xpos_old=xpos
    altitude_old=altitude
    zpos_old=zpos
    #  val = ds.getValue()
    fl_wheel=0
    fr_wheel=0
    rl_wheel=0
    rr_wheel=0
    ## Now we send some things to ros BELOW
#ROS#    camera_image_msg = Image()
#ROS#    camera_image_msg.width = 320
#ROS#    camera_image_msg.height = 240
#ROS#    camera_image_msg.encoding = "bgra8"
#ROS#    camera_image_msg.is_bigendian = 1
#ROS#    camera_image_msg.step = 1280
#ROS#    camera_image_msg.data = camera.getImage()
#ROS#    camera_pub.publish(camera_image_msg)
    
    
    ## Now we send some things to ros BELOW
#ROS#    rearcamera_image_msg = Image()
#ROS#    rearcamera_image_msg.width = 320
#ROS#    rearcamera_image_msg.height = 240
#ROS#    rearcamera_image_msg.encoding = "bgra8"
#ROS#    rearcamera_image_msg.is_bigendian = 1
#ROS#    rearcamera_image_msg.step = 1280
#ROS#    rearcamera_image_msg.data = rearcamera.getImage()
   
            
    #rearcamera_image_msg.data = flat_list
            
#ROS#    rearcamera_pub.publish(rearcamera_image_msg)
    
#ROS#    depth_msg.data = altitude
#ROS#    depth_pub.publish(depth_msg)
    
    radcoeff = 180.0/pi
    # Process sensor data here.
    #rospy.loginfo("Sending Simulated IMU Data. Roll: "+str(round(roll*radcoeff))+" Pitch: "+str(round(pitch*radcoeff))+" Heading: "+str(round(heading*radcoeff)))
#ROS#    pub.publish(Vector3(roll*radcoeff*-1,pitch*radcoeff*-1,heading*radcoeff*-1))
#ROS#    speed_pub.publish(Vector3(math.cos(heading)*xSpeed*-1+math.sin(heading)*zSpeed*-1,ySpeed,math.sin(heading)*xSpeed+math.cos(heading)*zSpeed))
#ROS#    log_pub.publish(str(round(roll*radcoeff))+","+str(round(pitch*radcoeff))+","+str(round(heading*radcoeff))+","+str(altitude)+","+str(roll_vel)+","+str(bleh)+","+str(pitch_vel)+","+str(xSpeed)+","+str(ySpeed)+","+str(zSpeed))
    
#ROS#    bleh_pub.publish(bleh)
    drive = 0
    side = 0
    car_trun=0
    yaw_stop=1
    key=KeyB.getKey()
    while (key>0):
        if (key==ord('W')):
            drive = 1
        if (key==ord('A')):
            side = 1
        if (key==ord('S')):
            drive = -1
        if (key==ord('D')):
            side = -1
        if (key==ord('Q')):
            yaw_disturbance = heading+0.174533
            car_trun=1
        if (key==ord('E')):
            yaw_disturbance = heading-0.174533
            car_trun=-1
        if (key==ord('Z')):
            target_altitude = altitude+0.8
            altitude_bool=True
        if (key==ord('X')):
            target_altitude = altitude-0.8
            altitude_bool=True
        if (key==ord('1')):
            drone_mode = True
            atache_mode = False
            car_mode = False
            print("Drone Mode")
        if (key==ord('2')):
            drone_mode = False
            atache_mode = True
            car_mode = False
            print("atache Mode")
        if (key==ord('3')):
            drone_mode = False
            atache_mode = False
            car_mode = True
            print("Car Mode")
        if (key==ord('4')):
            drone_mode = False
            atache_mode = False
            car_mode = False
        if (key==ord('N')):
            f = open("test2.txt", "w")
            f.write("xpos,ypos,zpos\n")
            logger=True
        if (key==ord('M')):
            f.close()
            logger=False
        key=KeyB.getKey()
    
    # Process sensor data here.
    if logger==True:
        f.write(str(xpos)+","+str(altitude)+","+str(zpos)+"\n")
    if (drone_mode==True):
            #print(str(roll)+"\t"+str(pitch)+"\t"+str(heading))
    #print(str(roll)+"\t"+str(roll_vel))
    #if abs(roll_vel_old-roll_vel)>2:
    #    k_roll_d=0
    #else:
    #    k_roll_d=0#10.0
        roll_input = k_roll_p * (angle_dist*side-roll) - k_roll_d*roll_vel
        pitch_input = (k_pitch_p *(angle_dist*drive-pitch) - k_pitch_d*pitch_vel)
        if abs(roll_input)>20 or abs(pitch_input)>20:
            yaw_stop=0
        if (yaw_disturbance>(math.pi)):
            yaw_disturbance=yaw_disturbance-2*math.pi
        elif (yaw_disturbance<-(math.pi)):
            yaw_disturbance=yaw_disturbance+2*math.pi
        yaw_error=yaw_disturbance-heading
        if (yaw_error>(math.pi)):
            yaw_error=yaw_error-2*math.pi
        elif (yaw_error<(-math.pi)):
            yaw_error=yaw_error+2*math.pi
        yaw_input = yaw_stop*(k_yaw_p*yaw_error- k_yaw_d*bleh);
    #print("pitch_input: "+str(pitch_input)+"\t velocity: "+str(pitch_vel))
    #print(str(yaw_disturbance)+"  \t"+str(heading)+"  \t"+str(yaw_error))
        vertical_input = k_vertical_p *CLAMP(target_altitude - altitude, -2.0, 2.0)-k_vertical_d*ySpeed;
        if roll>math.pi/2 or roll<-math.pi/2:
            vertical_input=-vertical_input
        if altitude_bool==True:
            target_altitude=altitude
            altitude_bool=False
    #print(str(vertical_input)+"\t"+str(target_altitude - altitude))
    #vertical_input = 0# k_vertical_p * pow(clamped_difference_altitude, 3.0);
    #0.2635 #0.266  #0.2635 #0.266  #roll distance
    #0.3582 #0.3582 #0.3346 #0.3346 #pitch distance
        #print(str(vertical_input)+"\t"+str(roll_input)+"\t"+str(pitch_input))
        front_left_motor_input =k_vertical_thrust + vertical_input + roll_input + (0.321/0.246)*pitch_input - yaw_input
        front_right_motor_input=k_vertical_thrust + vertical_input - roll_input + (0.321/0.246)*pitch_input + yaw_input
        rear_left_motor_input  =k_vertical_thrust + vertical_input + roll_input - (0.321/0.246)*pitch_input + yaw_input
        rear_right_motor_input =k_vertical_thrust + vertical_input - roll_input - (0.321/0.246)*pitch_input - yaw_input
    elif atache_mode==True:
        
        if not(abs(drive) or abs(side)):
            vertical_input = k_vertical_p *CLAMP(target_altitude - altitude, -2.0, 2.0)-k_vertical_d*ySpeed;
            if roll>math.pi/2 or roll<-math.pi/2:
                vertical_input=-vertical_input
            if altitude_bool==True:
                target_altitude=altitude
                altitude_bool=False
            lock_on=1
        else:
            vertical_input=0
            lock_on=-2
        roll_input = k_roll_p * (angle_lock*side-roll) - k_roll_d*roll_vel
        pitch_input = (k_pitch_p *(-1.48353*drive-pitch) - k_pitch_d*pitch_vel)
    #print("pitch_input: "+str(pitch_input)+"\t velocity: "+str(pitch_vel))
    #print(str(yaw_disturbance)+"  \t"+str(heading)+"  \t"+str(yaw_error))
        front_left_motor_input =lock_on*k_vertical_thrust + vertical_input + roll_input + (0.321/0.246)*pitch_input
        front_right_motor_input=lock_on*k_vertical_thrust + vertical_input - roll_input + (0.321/0.246)*pitch_input
        rear_left_motor_input  =lock_on*k_vertical_thrust + vertical_input + roll_input - (0.321/0.246)*pitch_input
        rear_right_motor_input =lock_on*k_vertical_thrust + vertical_input - roll_input - (0.321/0.246)*pitch_input
        #print(str(k_vertical_thrust)+"  \t"+str(roll_input))
    elif car_mode==True:
        front_left_motor_input =-2*k_vertical_thrust-car_trun*400
        front_right_motor_input=-2*k_vertical_thrust+car_trun*400
        rear_left_motor_input  =-2*k_vertical_thrust+car_trun*400
        rear_right_motor_input =-2*k_vertical_thrust-car_trun*400
        fl_wheel=4*drive+4*side
        fr_wheel=4*drive-4*side
        rl_wheel=4*drive+4*side
        rr_wheel=4*drive-4*side
        #print(str(k_vertical_thrust)+"  \t"+str(roll_input))
    else :
        front_left_motor_input =2*k_vertical_thrust
        front_right_motor_input=2*k_vertical_thrust
        rear_left_motor_input  =2*k_vertical_thrust
        rear_right_motor_input =2*k_vertical_thrust

    clampval = 10000
    #print(str(front_left_motor_input)+"\t"+str(front_right_motor_input)+"\t"+str(rear_left_motor_input)+"\t"+str(rear_right_motor_input))
    if front_left_motor_input>1000 or front_right_motor_input>1000 or rear_left_motor_input>1000 or rear_right_motor_input>1000:
        print("motor input maxed: "+str(int(time.time())))
    front_left_motor.setVelocity(CLAMP(front_left_motor_input,-clampval,clampval))#positive is up  #0.44467908653
    front_right_motor.setVelocity(CLAMP(-front_right_motor_input,-clampval,clampval))#negative is up #0.44616503673
    rear_left_motor.setVelocity(CLAMP(-rear_left_motor_input,-clampval,clampval))#negative is up     #0.42589835641
    rear_right_motor.setVelocity(CLAMP(rear_right_motor_input,-clampval,clampval))#positive is up  #0.42744959936
    #fly_wheel.setVelocity(spin_boy)
    FL_wheel.setVelocity(fl_wheel)
    FR_wheel.setVelocity(fr_wheel)
    RL_wheel.setVelocity(rl_wheel)
    RR_wheel.setVelocity(rr_wheel)
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    
    pass

# Enter here exit cleanup code.