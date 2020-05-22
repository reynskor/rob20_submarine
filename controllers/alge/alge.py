"""alge controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor
from controller import Display
import math
# create the Robot instance.
supervisor = Supervisor()

# do this once only




# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
display = supervisor.getDisplay("display");

width = display.getWidth();
height = display.getHeight();

# prepare stuff to get the
# Robot(IROBOT_CREATE).translation field
##WbNodeRef mybot = wb_supervisor_node_get_from_def("Submarine");
robot_node = supervisor.getFromDef("Submarine")
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")
# set the background (otherwise an empty ground is displayed at this step)
background = display.imageLoad("../../New folder for sam/alga.jpg");
display.imagePaste(background, 0, 0, False)#wb_display_image_paste(display, background, 0, 0, false);

# set the pen to remove the texture
display.setAlpha(0.0)#wb_display_set_alpha(display, 0.0);
brush_radius = 50
boat_x_location = 0
boat_y_location = 8
boat_x_size = 7
boat_y_size = 3
adsf=False
def translate(value, leftMin, leftMax, rightMin, rightMax): 
    # Figure out how 'wide' each range is 
    leftSpan = leftMax - leftMin 
    rightSpan = rightMax - rightMin 
    # Convert the left range into a 0-1 range (float) 
    valueScaled = float(value - leftMin) / float(leftSpan) 
    # Convert the 0-1 range into a value in the right range. 
    return rightMin + (valueScaled * rightSpan)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    x_pos, y_pos, z_pos = trans_field.getSFVec3f()#const double *translation = wb_supervisor_field_get_sf_vec3f(translationField);
    x_rot, y_rot, z_rot, angle = rot_field.getSFRotation()
    if z_pos>1.7 and abs(x_pos)<4 and ((abs(y_pos-8))<2):
        x_img=translate(x_pos,boat_x_location-boat_x_size/2,boat_x_location+boat_x_size/2,0,2048)
        y_img=translate(y_pos,boat_y_location-boat_y_size/2,boat_y_location+boat_y_size/2,0,1024)
    #print(str(translate(x_pos,-3.5,3.5,0,2048))+"\t"+str(translate(y_pos,3.5,6.5,0,1024)))
    #print(str(x_img)+"\t"+str(y_img))
    ##// display the robot position
    #display.drawRectangle(int(x_img)-150,int(y_img)-150,300,300)
        R11=(1-math.cos(angle))*x_rot*x_rot+math.cos(angle)
        R21=(1-math.cos(angle))*x_rot*y_rot+z_rot*math.sin(angle)
        q= math.atan2(R21,R11)
        brush_x_offset = int(brush_radius*math.sin(q))
        brush_y_offset = int(brush_radius*math.cos(q))
        display.fillOval(int(x_img)+brush_x_offset,int(y_img)-brush_y_offset,brush_radius,brush_radius)
        display.fillOval(int(x_img)-brush_x_offset,int(y_img)+brush_y_offset,brush_radius,brush_radius)
        adsf=True
        #display.imageSave(None,"clean.png")
    elif adsf==True:
        adsf=False
        #display.imageSave(None,"clean.png")
    #wb_display_fill_oval(display, width * (translation[X] + GROUND_X / 2) / GROUND_X,height * (translation[Z] + GROUND_Z / 2) / GROUND_Z, 7, 7);
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
