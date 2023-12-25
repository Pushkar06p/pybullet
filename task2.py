import gym
import os
import cv2
import time as t
import numpy as np
import pybullet_workshop_23
import math
import pybullet as p

CAR_LOCATION = [3,0,1.5]

BALL_LOCATION = [-3,0,1.5]

HUMANOID_LOCATION = [6,7,1.5]

VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 13,
    'cam_yaw'        : 0,
    'cam_pitch'      : -110,
    'cam_target_pos' : [0,4,0]
})

##########################################################
"""
Your Task is to grab the ball that is in front of the husky and then find the humanoid and shoot the ball.
"""
##########################################################

os.chdir(os.path.dirname(os.getcwd()))
# Environment Setup ::---
env = gym.make('pybullet_workshop_23',
               arena="arena2",
               car_location=CAR_LOCATION,
               ball_location=BALL_LOCATION,
               humanoid_location=HUMANOID_LOCATION,
               visual_cam_settings=VISUAL_CAM_SETTINGS
               )

###################### Write your code from here ###########################
while True:
    img = env.get_image(cam_height=0, dims=[512,512])
    k = cv2.waitKey(100)
    if k == ord('q'):
        break
   
    env.open_grip()
    env.move(vels=[[2,2],
                [2,2]])
    t.sleep(5.5)
    env.close_grip() 
    t.sleep(1)
    env.move(vels=[[6,-6],
                [6,-6]])


    while True:
                # Get the robot position and orientation
            
                pos, robot_orn = p.getBasePositionAndOrientation(env.car)

                # Calculate the direction vector from the robot to the target
                target_direction = [env.humanoid_location[0] - pos[0], env.humanoid_location[1] - pos[1]]
                target_angle = math.atan2(-target_direction[1],target_direction[0])

                # Check if the robot is pointing towards the target
                if abs(target_angle - robot_orn[2] + 1.732) < 0.1:
                    env.move(vels=[[0,0],
                                    [0,0]])
                    env.shoot()
                    t.sleep(4)
                    break

                
            
                p.stepSimulation()
                t.sleep(0.01)
