#! /usr/bin/env morseexec
from morse.builder import *

""" Building the 1st Robot 'r1' """
robo1 = ATRV()
robo1.translate(x=10, y=101, z=10) # Start with a jump :)
motion1 = MotionVW()     # Create a new instance of the actuator
motion1.translate(z=0.3) # Place the component at the specific location (x,y,z)
pOse1 = Pose()           # Create a new instance of the sensor
pOse1.translate(z=0.83)  # Place de component at the specific location (x,y,z)
robo1.append(motion1); robo1.append(pOse1);     # Appending Actuator and Sensor to 'r1'
# Configuring the middlewares of ALL the robot components: Data-streams & Services
robo1.add_default_interface('socket')
robo1.properties(Object = True, Graspable = False, Label = "Mouse")
controller1 = Keyboard()
controller1.properties(Speed = 4.0)
robo1.append(controller1)


""" Building the 2nd Robot 'r2' """
robo2 = B21()
robo2.translate(x=12, y=101, z=10) # Start with a jump and away from r1
motion2 = Waypoint()     # Create a new instance of the actuator
motion2.translate(z=0.3) # Place the component at the specific location (x,y,z)
B21pose2 = Pose()           # Create a new instance of the sensor
B21pose2.translate(z=0.83)  # Place de component at the specific location (x,y,z)
robo2.append(motion2); robo2.append(B21pose2);
camL = SemanticCamera()
camL.translate(x=0.2, y=0.3, z=0.9)
camR = SemanticCamera()
camR.translate(x=0.2, y=-0.3, z=0.9)
robo2.append(camL); robo2.append(camR);
robo2.add_default_interface('socket')

""" Choosing an Environment """
env = Environment('caylus50.blend')
env.place_camera([70, 70, 50])
env.aim_camera([0.7, 0.3, 0.315])
env.select_display_camera(camL)
