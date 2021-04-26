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
from svgpathtools import svg2paths2, path, Line, Arc
paths, attributes, svg_attributes = svg2paths2('geek.svg')

### BEGiN SET UP ###===========================================================

#Wheel speed info
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.5725 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10 # Left wheel index
MOTOR_RIGHT = 11 # Right wheel index
N_PARTS = 12 # Total joints'
IKPY_MAX_ITERATIONS = 4

# create the Robot instance.
supervisor = Supervisor()

# get the time step of the current world.
timeStep = int(4 * supervisor.getBasicTimeStep())

# Create the arm chain from the URDF
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))

armChain = Chain.from_urdf_file(filename)

#from Piazza post alters armChain but does not resolve error
# armChain = Chain.from_urdf_file(filename, base_elements = ["base_link", "base_link_Torso_joint", 
# "Torso", "torso_lift_joint", "torso_lift_link", 
# "torso_lift_link_TIAGo front arm_35411_joint", "TIAGo front arm_35411"])

for i in [0, 6]:
    armChain.active_links_mask[0] = False

print ("DEBUG: armchain \n", armChain)


# Initialize the arm motors and encoders.
print("DEBUG: armChain.links\n", armChain.links)
# arm_1_joint through arm_7_joint control arm
motors = []
for link in armChain.links:
    if 'motor' in link.name:
        motor = supervisor.getDevice(link.name)
        motor.setVelocity(1.0)
        position_sensor = motor.getPositionSensor()
        position_sensor.enable(timeStep)
        motors.append(motor)

# Initialize the arm motors and encoders.
# motors = []
# for link in armChain.links:
#     if 'joint' in link.name:
#         motor = supervisor.getDevice(link.name)
#         motor.setVelocity(1.0)
#         position_sensor = motor.getPositionSensor()
#         position_sensor.enable(timeStep)
#         motors.append(motor)

print ("DEBUG: notors \n", motors) 

# set targets
target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()

print("DEBUG: target = ", target)

print("DEBUG: arm = ", arm)

# gps = supervisor.getDevice("gps")
# gps.enable(timestep)
# compass.enable(timestep)
display = supervisor.getDevice("display")

# Defines the origin of the drawing board and scale for the SVG conversion
x_offset = 1.1-0.45
y_offset = -0.95+0.45
z_offset = 0.05
scale=80.0
### EnD SET UP ###===========================================================

### BEGiN LOAD SVG ###===========================================================

paths, attributes, svg_attributes = svg2paths2('geek.svg')

waypoints=[]
for path in paths:
    for element in path:
        if(isinstance(element,Line)):
            xs,ys,xe,ye = element.start.real,element.start.imag, element.end.real,element.end.imag, 
            waypoints.append(((xs/scale,ys/scale),(xe/scale,ye/scale)))

print ("DEBUG: waypoints\n", waypoints)

#prep fro sm
action=0
substep=0
steps=[]
state='init'
reached=[]

### EnD LOAD SVG ###===========================================================

# # The Tiago robot has multiple motors, each identified by their names below
# part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
#               "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
#               "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# # target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')             
# # target_pos = (0.0, 0.0, 0.09, 0.07, 0.26, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
# # target_pos = (0.0, 0.0, 0.00, 0.00, 0.0, 0, 0, 0, 0.0, 1.41, 'inf', 'inf')

# target_pos = (0.0, 0.0, 0.09, 0.07, -1.5, 0.4, 2.0, 2.0, -1.39, 0, 'inf', 'inf')             

# # target_pos = np.load("path.npy")
# # print(target_pos)

# robot_parts = []

# for i in range(N_PARTS):
#         robot_parts.append(supervisor.getDevice(part_names[i]))
#         robot_parts[i].setPosition(float(target_pos[i]))
#         robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# # Odometry
# pose_x     = 0
# pose_y     = 0
# pose_theta = 0


# floor_pen = supervisor.getDevice("floor_pen")
wall_pen = supervisor.getDevice("pen")
#initialize pen by turning them both off and turn on in wall/floor mode
# floor_pen.write(False)
wall_pen.write(False)



# vL = 0
# vR = 0

# multiplier = 10
# goals = np.load("path.npy")
# # print("goals",goals*multiplier, goals.shape)
# # print(goals[0])
# # print(goals[0][0])
# state = 0
# counter = 0
# turn_counter = 0

# mode = 'floor'
mode = 'wall'
# mode = 'manual'

#gains?? change this maybe?? 
# p1 = 1.5
# p2 = 3

# #Shiv: Printing the waypoints on the display (increase the display size if not visible. )
# aa = 0
# # display.setColor(0xFFFFFF)
# for line in goals*multiplier:
#     for waypoint in line:
#         display.drawPixel(int(waypoint[0]*30), int(waypoint[1]*30)) 




# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timeStep) != -1:
    # if mode == 'floor': 
    #     pose_y = gps.getValues()[2]
    #     pose_x = gps.getValues()[0]
    #     n = compass.getValues()
    #     rad = -((math.atan2(n[0], n[2]))-1.5708)
    #     pose_theta = rad
    #     # if pose_theta > 0:
    #         # pose_theta = -pose_theta
    
    #     floor_pen.write(True)    
    #     curr_line = goals[counter]
    #     x = curr_line[state][0]*multiplier
    #     y = curr_line[state][1]*multiplier
        
        
    #     # x = 2.142855
    #     # y = 2.8571400000000002
    #     # print(x, y, pose_x, pose_y)
        
    #     # print(x, y)
        
    #     # vL = MAX_SPEED/2
    #     # vR = MAX_SPEED/2
        
    #     rho = math.sqrt(((x - pose_x)**2) + ((y - pose_y)**2))# euclidian distance
    #     previous_aa = aa
    #     aa = math.atan2(y - pose_y, x - pose_x)
        
    #     # Shiv: Trying some hack to contain the angular error within a logical range
    #     # At one point, this becomes -3.13 from 2.14 suddenly and the robot turns in just the opposite direction
    #     if abs(previous_aa - aa) > math.pi:
    #         aa = -aa
        
    #     # Shiv: The compass also suddenly changes the pose_theta drastically and this is again a hack
    #     if pose_theta > math.pi:
    #         pose_theta -= 2*math.pi
    #     alpha = aa + pose_theta
        
    #     if rho < 0.5:
    #         state += 1
        
    #     if state >= 3:
    #         state = 1
    #         counter +=1
    #     print("goal and pose ", x, y, pose_x, pose_y)
    #     print(rho, alpha, aa, pose_theta)
    
    #     # xNew = p1*rho
    #     # dtheta = -p2*alpha  
    #     if abs(alpha)>0.5:
    #         xNew = 0
    #         dtheta = -10*alpha
    #     else:
    #         xNew = 5*rho
    #         dtheta = -5*alpha
        
    #     vL = (xNew - (dtheta*(AXLE_LENGTH/2.)))
    #     vR = (xNew + (dtheta*(AXLE_LENGTH/2.)))   
    #     # print(vL, vR)
    #     if vL > MAX_SPEED/4:
    #         vL = MAX_SPEED/4
    #     elif vL < -MAX_SPEED/4:
    #         vL = -MAX_SPEED/4
    
    #     if vR > MAX_SPEED/4:
    #         vR = MAX_SPEED/4
    #     elif vR < -MAX_SPEED/4 :
    #         vR = -MAX_SPEED/4
    #     # pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    #     # pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    #     # pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
        
    #     # #pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    #     # #pose_y += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    #     # #pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0
        
    #     if counter > 7:
    #         vL = 0
    #         vR = 0
    #         counter = 0

    #     robot_parts[MOTOR_LEFT].setVelocity(vL)
    #     robot_parts[MOTOR_RIGHT].setVelocity(vR)

    ### BEGIN IK ###===========================================================
    if mode == 'wall':
      print("DEGUB: Now in wall mode")

      print("DEGUB: Entering state machine")
      print ("DEBUG: enter state = ", state)
    ### BEGiN State Machine  ###===========================================================
    #state machine from working example, will need to change it but can't until error is resolved

      if(state=='init'):
        x=x_offset
        y=y_offset
        z=z_offset
        if(all(reached)): 
            state='nextpath'
            supervisor.getDevice('pen').write(True)
            reached=False
      elif state=='substep':
          x=steps[0][substep]+x_offset
          y=-steps[1][substep]+y_offset
          z=z_offset
          #print((x,y,substep,len(steps[0])))
          if(all(reached)):
              substep+=1
          if(substep>=len(steps[0])):
              state='nextpath'
      elif state=='done':
          x=x_offset
          y=y_offset
          z=1
      elif state=='nextpath': 
          c=waypoints[action]
          #print("Line from (%f,%f) to (%f,%f)" % (c[1][0]+x_offset,-c[1][1]+y_offset,c[2][0]+x_offset,-c[2][1]+y_offset))
          display.setColor(0xFFFFFF)
          display.drawLine(int(c[0][0]*160),int(c[0][1]*160),int(c[1][0]*160),int(c[1][1]*160))  
          distance=math.sqrt((c[0][0]-c[1][0])**2+(c[0][1]-c[1][1])**2)
          npoints=int(distance/0.01)
          #print("Distance: %f npoints: %d" % (distance,npoints))
          if(npoints>=2):
              steps=[np.linspace(c[0][0],c[1][0],npoints),np.linspace(c[0][1],c[1][1],npoints)]
              state='substep'
          else:
              steps=[[c[1][0]],[c[1][1]]]
          x=steps[0][0]+x_offset
          y=-steps[1][0]+y_offset
          z=z_offset
          substep=1 
          if(all(reached)):
              action+=1
          if(action>=len(waypoints)):
              state='done'
      ### EnD State MAchine ###===========================================================
      print ("DEGUB: State machine traversed") 
      print ("DEBUG: exit state = ", state)

      ### BEGiN ACTUATE ###===========================================================
      # print("DEBUG: motots\n", motors)
      current_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
      print ("DEBUG: Current_position = ", current_position)
      print("DEBUG: x = ", x,", y = ", y,", z = ", z)
      ikResults = armChain.inverse_kinematics([x, y, z], max_iter=IKPY_MAX_ITERATIONS, initial_position=current_position)
      print ("DEBUG: ik calculated", ikResults)

      # Actuate the 3 first arm motors with the IK results.
      reached=[]
      for i in range(3):
          #### This line checks whether the robot did arrive
          reached.append(abs(motors[i].getPositionSensor().getValue()-ikResults[i + 1])<0.01)
          motors[i].setPosition(ikResults[i + 1])
      
      print("DEBUG: reached \n", reached)

      # Keep the hand orientation down.
      motors[4].setPosition(-ikResults[2] - ikResults[3] + math.pi / 2)
      # Keep the hand orientation perpendicular.
      motors[5].setPosition(ikResults[1])


        ### EnD ACTUATE ###===========================================================
       # wall_pen.write(True)
       #  print("Now in wall mode")
       #  turn_counter += 1
       # #turn the robot to go towards the wall
       #  if turn_counter < 30: 
       #      vL=MAX_SPEED/2
       #      vR=-MAX_SPEED/2
           
       #  if turn_counter > 30 and turn_counter < 440:
       #      vL=MAX_SPEED/2
       #      vR=MAX_SPEED/2
           
       #  if turn_counter >= 440: 
       #      wall_pen.write(True)
       #      vL = 0
       #      vR = 0
            
       #  t = supervisor.getTime()    
       #  # Use the circle equation relatively to the arm base as an input of the IK algorithm.
       #  #going to have to change this to iterate through svg points once we get this working
       #  x = 0.25 * math.cos(0) + 1.1
       #  y = 0.25 * math.sin(0) - 0.95
       #  z = 0.05

       #  #comput arm ik
       #  initial_position = [0] + [m.getPositionSensor().getValue() for m in motors] + [0]
       #  ikResults = armChain.inverse_kinematics([x,y,z], max_iter = IKPY_MAX_ITERATIONS, initial_position = initial_position)

       #  #use IK to activate motors
       #  for i in range(3):
       #    motors[i].setPosition(ikResults[i+1])

       #  #NOT SURE WHAT THIS IS LOOK INTO 
       #  motors[3].setPosition(-ikResults[2] - ikResults[3] + math.pi/2)
       #  motors[5].setPosition(ikResults[1])

       #  #set up exit conditions TEMPORARY UNTIL COMBINED WITH MAIN LOOP
       #  if supervisor.getTime() > 2 * math.pi + 1.5:
       #    break #timout
       #  elif supervisor.getTime() > 1.5:
       #    supervisor.getDevice('pen').write(True)


        ### EnD IK ###===========================================================

    # if mode == 'manual': 
    #     print('manual')
# Enter here exit cleanup code.
