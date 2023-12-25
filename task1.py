## Library Imports :: ---
import gym
import os
import cv2
import pybullet_workshop_23
import time as t
import pybullet as p

CAR_LOCATION = [2,3,1.5]
# BALL_LOCATION = [4,5,1.5]
# HUMANOID_LOCATION = [-3,-4,1.3]
VISUAL_CAM_SETTINGS = dict({
    'cam_dist'       : 13,
    'cam_yaw'        : 0,
    'cam_pitch'      : -110,
    'cam_target_pos' : [0,4,0]
})

##########################################################
"""
Your task is to use env.move() and p.getKeyboardEvents() to control the husky using the arrow keys
of you keyboard and make the husky stop when the spacebar is pressed just like games :D
"""
##########################################################

os.chdir(os.path.dirname(os.getcwd()))

#Environment Setup ::---
env = gym.make('pybullet_workshop_23',
    arena = "default",
    car_location=CAR_LOCATION,
    visual_cam_settings=VISUAL_CAM_SETTINGS
)


###################### Write your code from here ###########################
while True:
    img = env.get_image(cam_height=0, dims=[512,512])
    k = cv2.waitKey(0)
    cv2.imshow("img",img)
    if k == ord('q'):
        
        break
    while (1):
     keys = p.getKeyboardEvents()
     for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
          env.move(vels=[[2,2],
               [2,2]])
          t.sleep(0.1)  

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            env.move(vels=[[-2,2],
               [-2,2]])
            t.sleep(0.1)  


        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            env.move(vels=[[-2,-2],
               [-2,-2]])
            t.sleep(0.1)  

           

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
           env.move(vels=[[2,-2],
               [2,-2]])
           t.sleep(0.1) 

        if(32 in keys and keys[32] == 1):
           env.move(vels=[[0,0],
               [0,0]])
           t.sleep(0.1)