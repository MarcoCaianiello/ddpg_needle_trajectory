#!/usr/bin/env python3
import numpy as np
pi = 3.14159265358979

#0.0993763 -0.0755887 -0.129786 -1.91247 0.122941 -2.17051

#Start State
startPosition = np.array([0.0994122, -0.0756321, -0.129774]) #sim
startEuler = np.array([-1.91247, 0.140941, -2.27051]) #sim

#startPosition = np.array([-0.0480892, -0.0288379, -0.0741711]) #real
#startEuler = np.array([2.97941, -0.551759, -2.52409]) #real

#startPosition = np.array([-0.08662988, -0.0837255,  -0.13229138]) #real
#startEuler = np.array([0.51185741, -1.46067898, -0.48132047]) #real

#Goal State 
goalPosition = np.array([0.1279,-0.0750,-0.1361]) #sim
goalEuler = np.array([-pi/2, 0.116652, -2.16378]) #sim

#goalPosition = np.array([-0.1, -0.0288379, -0.15]) #real
#goalEuler = np.array([2.97941, -pi/2, -2.52409]) #real
#goalEuler = np.array([0.582277, -pi/2, -0.57]) #real

#Joint limit
q_max = [pi/3, pi/2, 0.25, pi, pi/2, pi/2]
q_min = [-pi/3, -pi/2, 0.05, -pi, -pi/2, -pi/2]

# State space setup
# Position and Orientation of the needle tip
num_states = 6

# Action space setup
# Position and Orientation variation
num_actions = 6

#Min&Max Variation
max_distance = 0.01  #0.01
min_distance = -0.01 #-0.01

#Goal Threshold
goalPosTh = 0.001 #m
goalOrientTh = 0.0873 #5°

#Out of bounds Threshold
outTh = 5

#Standard Deviation for OUActionNoise
std_dev = 0.5*max_distance #0.2

#Total number of episode
total_episodes = 300

#Max number of actions
max_actions = 2.5e2

# Discount factor for future rewards
gamma = 0.99

# Used to update target networks
tau = 0.01 #0.005

# Buffer setup
buffer_capacity=50000
batch_size=100

# Learning rate for actor-critic models
critic_lr = 1e-2 #1e-3
actor_lr = 5e-3 #1e-3

# Probability to sample from demostration buffer
eps_dem = 0.4

# Probability to sample from experience buffer
eps_exp = 1 - eps_dem

# Variation of eps_dem and eps_exptrajectory_dem_04_1
delta_dem = 0.003
delta_exp = 0.1

# To store reward history of each episode
ep_reward_list = []
# To store average reward history of last few episodes
avg_reward_list = []

#Weight path
weight_path="/home/marco/Desktop/ros_ws/src/dvrk_rl/weight/"

#Trajectory path
file_path= "/home/marco/Desktop/ros_ws/src/dvrk_rl/dem_buffer/"

#Reward path
rwd_path= "/home/marco/Desktop/ros_ws/src/dvrk_rl/dem_buffer/reward/"
