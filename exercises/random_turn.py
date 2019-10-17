#!/usr/bin/env python3


import time
import sys
import argparse
import math
import numpy as np
import gym
import cv2
import time
import random
from gym_duckietown.envs import DuckietownEnv
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--env-name', default=None)
parser.add_argument('--map-name', default='4way')
parser.add_argument('--no-pause', action='store_true', help="don't pause on failure")
parser.add_argument('--mask-enable', action='store_true')
args = parser.parse_args()

if args.env_name is None:
    env = DuckietownEnv(
        map_name = args.map_name,
        domain_rand = False,
        draw_bbox = False,
        draw_curve = False
    )
else:
    env = gym.make(args.env_name)

obs = env.reset()
env.render()
mask_enable = args.mask_enable
if mask_enable:
    cv2.startWindowThread()
    cv2.namedWindow('Red_Lines_Mask')


#Parametro impostabile per il tempo da aspettare in secondi negli incroci senza semaforo
TIME_TO_WAIT = 2


total_reward = 0
(height, width, depth) = obs.shape
at_Stop_Line = False
time_after_stop = 20
lower_red = np.array([140,20,20])
upper_red = np.array([255,120,140])
turn_right = False
turn_left = False
time_at_curve = 0
while True:

    k_p = 40            #Default=40                          60
    k_d = 10            #Defautl=10                          30
    speed = 0.3         #Default=0.3                   max = 1
    
    if turn_right:
        i, j = env.get_grid_coords(env.cur_pos)
        x, _, z = env.get_dir_vec()
        x = round(x)
        z = round(z)
        j_rounded = j + z
        i_rounded = i + x
        direction2 = None
        print(x,z, 'Direzione presa')
        if x == 0 and z == 1:
            direction2 = 'S'
        elif x == 0 and z == -1:
            direction2 = 'N'
        elif x == 1 and z == 0:
            direction2 = 'E'
        elif x == -1 and z == 0:
            direction2 = 'O'
        while i != i_rounded or j != j_rounded:
            lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
            distance_to_road_center = lane_pose.dist
            angle_from_straight_in_rads = lane_pose.angle_rad
            steering = k_p*distance_to_road_center + k_d*angle_from_straight_in_rads
            obs, reward, done, info = env.step([speed, steering])
            total_reward += reward
            print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))
            env.render()
            i, j = env.get_grid_coords(env.cur_pos)

            mask = cv2.inRange(obs, lower_red, upper_red)
            #Immagine della maschera su schermo
            if mask_enable:
                cv2.imshow('Red_Lines_Mask',mask)
                cv2.waitKey(1)
        starting_angle = env.cur_angle
        #Qui viene gestita la svolta a destra
        while i == i_rounded and j == j_rounded:
            tile = env._get_tile(i,j)
            kind = tile['kind']
            if kind.startswith('3way'):
                if(env.cur_angle < (starting_angle - np.pi*3/8)):
                    break
            """if kind.startswith('4way'):
                if(env.cur_angle < (starting_angle - np.pi*7/20)):
                    break"""
            lane_pose = env.get_right_turn(env.cur_pos, env.cur_angle, direction=direction2)
            distance_to_road_center = lane_pose.dist
            angle_from_straight_in_rads = lane_pose.angle_rad
            steering = k_p*distance_to_road_center + k_d*angle_from_straight_in_rads
            obs, reward, done, info = env.step([speed, steering])
            total_reward += reward
            print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))
            env.render()
            i, j = env.get_grid_coords(env.cur_pos)

            mask = cv2.inRange(obs, lower_red, upper_red)
            #Immagine della maschera su schermo
            if mask_enable:
                cv2.imshow('Red_Lines_Mask',mask)
                cv2.waitKey(1)
        turn_right = False
        continue
        
    if turn_left:
        i, j = env.get_grid_coords(env.cur_pos)
        x, _, z = env.get_dir_vec()
        j_rounded = j + round(z)
        i_rounded = i + round(x)
        while i != i_rounded or j != j_rounded:
            lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
            distance_to_road_center = lane_pose.dist
            angle_from_straight_in_rads = lane_pose.angle_rad
            steering = k_p*distance_to_road_center + k_d*angle_from_straight_in_rads
            obs, reward, done, info = env.step([speed, steering])
            total_reward += reward
            print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))
            env.render()
            i, j = env.get_grid_coords(env.cur_pos)

            mask = cv2.inRange(obs, lower_red, upper_red)
            #Immagine della maschera su schermo
            if mask_enable:
                cv2.imshow('Red_Lines_Mask',mask)
                cv2.waitKey(1)
        starting_angle = env.cur_angle
        #Qui viene gestita la svolta a sinistra
        while i == i_rounded and j == j_rounded:
            if kind.startswith('3way'):
                if(env.cur_angle > (starting_angle + np.pi*3/8)):
                    break
            if kind.startswith('4way'):
                if(env.cur_angle > (starting_angle + np.pi*3/9)):
                    break
            lane_pose = env.get_left_turn(env.cur_pos, env.cur_angle)
            distance_to_road_center = lane_pose.dist
            angle_from_straight_in_rads = lane_pose.angle_rad
            steering = k_p*distance_to_road_center + k_d*angle_from_straight_in_rads
            obs, reward, done, info = env.step([speed, steering])
            total_reward += reward
            print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))
            env.render()
            i, j = env.get_grid_coords(env.cur_pos)

            mask = cv2.inRange(obs, lower_red, upper_red)
            #Immagine della maschera su schermo
            if mask_enable:
                cv2.imshow('Red_Lines_Mask',mask)
                cv2.waitKey(1)
        turn_left = False
        continue




    lane_pose = env.get_lane_pos2(env.cur_pos, env.cur_angle)
    distance_to_road_center = lane_pose.dist
    angle_from_straight_in_rads = lane_pose.angle_rad

    # angle of the steering wheel, which corresponds to the angular velocity in rad/s
    steering = k_p*distance_to_road_center + k_d*angle_from_straight_in_rads 


    #Filtraggio del Rosso    
    mask = cv2.inRange(obs, lower_red, upper_red)
    
    #Immagine della maschera su schermo
    if mask_enable:
        cv2.imshow('Red_Lines_Mask',mask)
        cv2.waitKey(1)
    

    if not at_Stop_Line and time_after_stop > 5/speed:           #Scansione alla ricerca della linea di stop
        lines = cv2.HoughLines(mask,1,np.pi/180,80)
        if lines is not None:
            for line in lines:
                for rho, theta in line:
                    if rho > 530 and (theta < np.pi*7/10 and theta > np.pi*3/10) :      #rho > 570
                        at_Stop_Line = True
                        #print(rho,theta)

    if at_Stop_Line:
         semaforo_individuato = False
         semaforo_verde = False
         grayscale = cv2.cvtColor(obs,cv2.COLOR_RGB2GRAY)
         if mask_enable:
              cv2.imshow('Grigio',grayscale)
         circles = cv2.HoughCircles(grayscale, cv2.HOUGH_GRADIENT,5,10,300,10,minRadius=10,maxRadius=30)     #2,30,300,20, min 0, max 30
         if circles is not None:
             circles = np.round(circles[0, :]).astype("int")
             for (x,y,r) in circles:
               if y < 70 and x <220:
                 (red, green, blue) = obs[y,x]
                 if red < 30 and green > 100 and blue < 30:
                    print('Semaforo verde all\'arrivo')
                    #cv2.circle(grayscale, (x,y), r, (0,255,0), -1)
                    #if mask_enable:
                    #    cv2.imshow('Grigio',grayscale)
                    semaforo_individuato = True
                    semaforo_verde = True
                 elif red>100 and green < 30 and blue <30:
                    print('semafero rosso')
                    #cv2.circle(grayscale, (x,y), r, (255,0,0), -1)
                    #if mask_enable:
                    #    cv2.imshow('Grigio',grayscale)
                    semaforo_individuato = True
                    semaforo_verde = False
         if semaforo_individuato and semaforo_verde:
             print('Proseguire')
         elif semaforo_individuato and not semaforo_verde:
             while  not semaforo_verde:
                 obs, reward, done, info = env.step([0,0])
                 total_reward += reward
                 print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))
                 env.render()
                 grayscale = cv2.cvtColor(obs,cv2.COLOR_RGB2GRAY)
                 circles = cv2.HoughCircles(grayscale, cv2.HOUGH_GRADIENT,5,10,300,20,minRadius=10,maxRadius=30)     #2,50,300,20
                 if circles is not None:
                     circles = np.round(circles[0, :]).astype("int")
                     for (x,y,r) in circles:
                         if y < 70 and x <220:
                             (red, green, blue) = obs[y,x]
                             if red < 30 and green > 100 and blue < 30:
                                 print('Semaforo diventato verde')
                                 semaforo_verde = True
         else:
             restart_time = time.time() + TIME_TO_WAIT
             while time.time() < restart_time:                     #Se non è rilevato un semaforo aspetta un numero impostabile di secondi
                 obs, reward, done, info = env.step([0,0])
                 total_reward += reward
                 print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))
                 env.render()
         at_Stop_Line = False
         time_after_stop = 0
         i, j = env.get_grid_coords(env.cur_pos)
         x, _, z = env.get_dir_vec()
         j_rounded = j + round(z)
         i_rounded = i + round(x)
         print(round(x),round(z))
         tile = env._get_tile(i_rounded, j_rounded)
         kind = tile['kind']
         print(kind)

         #Esaminazione direzioni possibili
         if kind.startswith('4way'):
             number=random.randint(1,3)
             print(number)
             if number == 1:
                 turn_left = True
             elif number == 2:
                 turn_right = True
         if kind.startswith('3way_left'):  #Restart to code here
             number : int
             angle = tile['angle']
             print(angle)
             if angle == 0:
                 if round(x)==0 and round(z)==1:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_left = True
                 if round(x)==0 and round(z)==-1:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
                 if round(x)==-1 and round(z)==0:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
             elif angle == 1:
                 if round(x)==0 and round(z)==1:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
                 if round(x)==1 and round(z)==0:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_left = True
                 if round(x)==-1 and round(z)==0:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
             elif angle == 2:
                 if round(x)==0 and round(z)==1:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
                 if round(x)==0 and round(z)==-1:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_left = True
                 if round(x)==1 and round(z)==0:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
             elif angle == 3:
                 if round(x)==1 and round(z)==0:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
                 if round(x)==-1 and round(z)==0:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_left = True
                 if round(x)==0 and round(z)==-1:
                     number = random.randint(1,2)
                     if number == 1:
                         turn_right = True
             print(number)

    else:
        time_after_stop += 1
    
    obs, reward, done, info = env.step([speed, steering])
    total_reward += reward
    
    print('Steps = %s, Timestep Reward=%.3f, Total Reward=%.3f' % (env.step_count, reward, total_reward))

    env.render()

    if done:
        if reward < 0:
            print('*** CRASHED ***')
        print ('Final Reward = %.3f' % total_reward)
        break


