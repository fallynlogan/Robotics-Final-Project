"""final_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import  Robot, Motor, Camera, RangeFinder, Lidar
import numpy as np
import ikpy
import math
import sys
import tempfile
from ikpy.chain import Chain
from controller import Supervisor

#Wheel speed info
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.5725 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10 # Left wheel index
MOTOR_RIGHT = 11 # Right wheel index
N_PARTS = 12 # Total joints
## are we gonna need this? 
IKPY_MAX_ITERATIONS = 4

# create the Robot instance.
robot = Robot()


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")
              
#target_pos = (0.0, 0.0, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
target_pos = (0.0, 0.0, 0.00, 0.00, 0.0, 0, 0, 0, 0.0, 1.41, 'inf', 'inf')

# target_pos = np.load("path.npy")
# print(target_pos)

robot_parts = []

for i in range(N_PARTS):
        robot_parts.append(robot.getDevice(part_names[i]))
        robot_parts[i].setPosition(float(target_pos[i]))
        robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

#setup robot supervisor @ https://www.cyberbotics.com/doc/reference/supervisor
# supervisor = Supervisor() -> 'leads to error Only one instance of the Robot class should be created'
#NEED TO CHANGE THIS *** ?
timeStep = int(4 * robot.getBasicTimeStep())


filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(robot.getUrdf().encode('utf-8'))
    
#create arm chain from urdf file    
armChain = Chain.from_urdf_file(filename)
#set active link mask
for i in [0, 6]:
    armChain.active_links_mask[0] = False
    
#initialize arms motors 
motors = []
for link in armChain.links:
  if 'motor' in link.name:
    motor = robot.getDevice(link.name)
    #may need a different velocity
    motor.setVelocity(1.0)
    position_sensor = motor.getPositionSensor()
    position_sensor.euclidian(timeStep)
    motor.append(motor)

#set arm and target node
# arm = robot.getSelf()
target = robot.getFromDef('TARGET')

floor_pen = robot.getDevice("floor_pen")
wall_pen = robot.getDevice("pen")
#initialize pen by turning them both off and turn on in wall/floor mode
floor_pen.write(False)
wall_pen.write(False)

vL = 0
vR = 0

floor_goals = np.load("path.npy")
manual_goals = np.load("manual_points.npy")

state = 0
floor_counter = 0
manual_counter = 0
turn_counter = 0

multiplier = 1.5

# mode = 'floor'
mode = 'wall'
# mode = 'manual'

#gains?? change this maybe?? 
p1 = 1.5
p2 = 3

# ### DRAW A CIRCLE ###
# print("Attempt to Draw a circle")
# #TEMPORARY UNTIL COMBINED WITH MAIN LOOP
# while supervisor.step(timeStep) != -1:
#   tim = supervisor.getTime()

#   #equation for the cirlce ----> loook into
#   x = 0.25 * math.cos(tim) + 1.1
#   y = 0.25 * math.sin(tim) -0.95
#   z = 0.05

#   #comput arm ik
#   initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
#   ikResults = armChain.inverse_kinematics([x,y,z], max_iter = IKPY_MAX_ITERATIONS, initial_position = initial_position)


#   #use IK to activate motors
#   for i in range(3):
#     motors[i].setPosition(ikResults[i+1])

#   #NOT SURE WHAT THIS IS LOOK INTO 
#   motors[3].setPosition(-ikResults[2] - ikResults[3] + math.pi/2)
#   motors[5].setPosition(ikResults[1])

#   #set up exit conditions TEMPORARY UNTIL COMBINED WITH MAIN LOOP
#   if supervisor.getTime() > 2 * math.pi + 1.5:
#     break #timout
#   elif supervisor.getTime() > 1.5:
#     supervisor.getDevice('pen').write(True)



# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # ### FLOOR MODE ###
    # if mode == 'floor': 
    #     floor_pen.write(True)    
    #     curr_line = floor_goals[floor_counter]
    #     # print("state_1: ", state, "counter", counter)
    #     print("x: ", curr_line[state][0]*multiplier, "y: ", curr_line[state][1]*multiplier)
    #     x = curr_line[state][0]*multiplier
    #     y = curr_line[state][1]*multiplier
        
    #     vL = MAX_SPEED/2
    #     vR = MAX_SPEED/2
        
    #     rho = math.sqrt(((pose_x - x)**2) + ((pose_y - y)**2))# euclidian distance
    #     alpha = math.atan2(y - pose_y, x - pose_x) - pose_theta
        
    #     if rho <= 0.35:
    #         state += 1
        
    #     if state > 99:
    #         state = 0
    #         floor_counter +=1
    
    #     x = p1*rho
    #     theta = -p2*alpha  
        
    #     vL = (x - (theta*(AXLE_LENGTH/2)))
    #     vR = (x + (theta*(AXLE_LENGTH/2)))   
        
        
    #     if(vL>MAX_SPEED/2):
    #         vL=MAX_SPEED/2
    #     elif(vL<-MAX_SPEED/2):
    #         vL=-MAX_SPEED/2
    
    #     if(vR>MAX_SPEED/2):
    #         vR=MAX_SPEED/2
    #     elif(vR<-MAX_SPEED/2):
    #         vR=-MAX_SPEED/2
        
    #     pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    #     pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    #     pose_theta += (vL-vR)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
    #     print("X: %f Y: %f Theta: %f" % (pose_x, pose_y, pose_theta))
        
    #     if floor_counter > 7:
    #         vL = 0
    #         vR = 0
    #         floor_counter = 0

    #     robot_parts[MOTOR_LEFT].setVelocity(vL)
    #     robot_parts[MOTOR_RIGHT].setVelocity(vR)

    ### WALL MODE ###
    if mode == 'wall': 
        print("Now in wall mode")
        turn_counter += 1
       #turn the robot to go towards the wall
        if turn_counter < 30: 
            vL=MAX_SPEED/2
            vR=-MAX_SPEED/2
           
        if turn_counter > 30 and turn_counter < 440:
            vL=MAX_SPEED/2
            vR=MAX_SPEED/2
           
        if turn_counter >= 440: 
            wall_pen.write(True)
            vL = 0
            vR = 0
            
        t = robot.getTime()    
        # Use the circle equation relatively to the arm base as an input of the IK algorithm.
        #going to have to change this to iterate through svg points once we get this working
        x = 0.25 * math.cos(0) + 1.1
        y = 0.25 * math.sin(0) - 0.95
        z = 0.05

        #comput arm ik
        initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
        ikResults = armChain.inverse_kinematics([x,y,z], max_iter = IKPY_MAX_ITERATIONS, initial_position = initial_position)

        #use IK to activate motors
        for i in range(3):
          motors[i].setPosition(ikResults[i+1])

        #NOT SURE WHAT THIS IS LOOK INTO 
        motors[3].setPosition(-ikResults[2] - ikResults[3] + math.pi/2)
        motors[5].setPosition(ikResults[1])

        #set up exit conditions TEMPORARY UNTIL COMBINED WITH MAIN LOOP
        if robot.getTime() > 2 * math.pi + 1.5:
          break #timout
        elif robot.getTime() > 1.5:
          robot.getDevice('pen').write(True)

    
        
    # ### MANUEL MODE ###   
    # if mode == 'manual': 
       
    #    points = manual_goals[manual_counter]
    #    x, y, z = points 
       
    #    turn_counter += 1
    #    #turn the robot to go towards the wall
    #    if turn_counter < 30: 
    #        vL=MAX_SPEED/2
    #        vR=-MAX_SPEED/2
           
    #    if turn_counter > 30 and turn_counter < 440:
    #        vL=MAX_SPEED/2
    #        vR=MAX_SPEED/2
           
    #    if turn_counter >= 440: 
    #        wall_pen.write(True)
    #        vL = 0
    #        vR = 0
       
       
    #    robot_parts[MOTOR_LEFT].setVelocity(vL)
    #    robot_parts[MOTOR_RIGHT].setVelocity(vR)
# Enter here exit cleanup code.
