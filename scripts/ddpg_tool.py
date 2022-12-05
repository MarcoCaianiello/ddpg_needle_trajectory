#!/usr/bin/env python3

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
import pickle
import matplotlib.pyplot as plt

import ddpg_param as prm

def save_weights (net):
    # Save the weights
    net.actor_model.save_weights(prm.weight_path+"needle_insert_actor.h5")
    net.critic_model.save_weights(prm.weight_path+"needle_insert_critic.h5")

    net.target_actor.save_weights(prm.weight_path+"needle_insert_target_actor.h5")
    net.target_critic.save_weights(prm.weight_path+"needle_insert_target_critic.h5")

def load_weights (net):
    # Save the weights
    net.actor_model.load_weights(prm.weight_path+"needle_insert_actor.h5")
    net.critic_model.load_weights(prm.weight_path+"needle_insert_critic.h5")

    net.target_actor.load_weights(prm.weight_path+"needle_insert_target_actor.h5")
    net.target_critic.load_weights(prm.weight_path+"needle_insert_target_critic.h5")

    return net


# Plotting graph
# Episodes versus Avg. Rewards
def plot_average_reward (avg_reward_list):
    plt.plot(avg_reward_list)
    plt.xlabel("Episode")
    plt.ylabel("Epsiodic Reward")
    plt.show()


# Mean of last x episodes
def mean_each_x_episodes (ep_reward_list, x):
    x = -1*x
    return np.mean(ep_reward_list[x:])


def save_on_file (list, name):
    with open(prm.file_path + name, 'wb') as file_handler:
        pickle.dump(list, file_handler)

def read_from_file (name):
    with open(prm.file_path + name, 'rb') as file_handler:
        return pickle.load(file_handler)


def convert_to_PoseMsg (data):
    msg = Pose()

    msg.position.x = data[0]
    msg.position.y = data[1]
    msg.position.z = data[2]
    msg.orientation.x = data[3]
    msg.orientation.y = data[4]
    msg.orientation.z = data[5]
    msg.orientation.w = 0

    return msg
#end_convert_to_PoseMsg

def convert_to_JointStateMsg (data):
    msg = JointState()

    msg.position = data

    return msg
#end_convert_to_PoseMsg

def abs_buffer (data):
    buffer = []

    for x in data:
        if ( x<0 ):
            buffer.append(-1)
        else:
            buffer.append(1)

    return np.array(buffer)

def divide (num, den):
    data = np.zeros((len(den)))

    if (np.isscalar(num)):
        for i in range(len(den)):
            if (den[i] != 0):
                data[i] = num/den[i]
            else:
                data[i] = 0

    elif (len(num)==len(den)):
        for i in range(len(den)):
            if (den[i] != 0):
                data[i] = num[i]/den[i]
            else:
                data[i] = 0

    return data


def print_round_array (data):
    s = "["
    for x in data:
        s = s + " " + "{:.4f}".format(x)
    s = s + " ]"
    return s


"""
To implement better exploration by the Actor network, we use noisy perturbations.
Ornstein-Uhlenbeck process for generating noise.
"""
class OUActionNoise:
    def __init__(self, mean, std_deviation, theta=0.15, dt=1e-2, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = std_deviation
        self.dt = dt
        self.x_initial = x_initial
        self.reset()

        for i in range(200):
          self.__call__()
        

    def __call__(self):
        # Formula taken from https://www.wikipedia.org/wiki/Ornstein-Uhlenbeck_process.
        x = (
            self.x_prev
            + self.theta * (self.mean - self.x_prev) * self.dt
            + self.std_dev * np.sqrt(self.dt) * np.random.normal(size=self.mean.shape)
        )
        # Store x into x_prev
        # Makes next noise dependent on current one
        self.x_prev = x
        return x

    def reset(self):
        if self.x_initial is not None:
            self.x_prev = self.x_initial
        else:
            self.x_prev = np.zeros_like(self.mean)

#ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=float(prm.std_dev) * np.ones(1), theta=0.5, dt=0.05)
ou_noise = OUActionNoise(mean=np.zeros(prm.num_states), std_deviation=float(prm.std_dev) * np.ones(prm.num_states))


# TO BE CHECKED
def noise_object (noise, episode, info, useOUA_Noise=True):

    temp = np.random.uniform(-prm.std_dev, prm.std_dev)
    delta_noise = 2 #0.003
    
    if (info == "OUT OF BOUND"):
        ep = (episode/2) - 15
    else:
        ep = episode

    if useOUA_Noise:
        temp = noise()
        #for i in range(len(temp)):
        #    temp[i]=temp[i]*delta_noise/(ep/15)

    '''
    for i in range(len(temp)):
        if temp[i] >= 0: 
            temp[i] = max(temp[i]-((ep/10)*delta_noise), 0)
        else:
            temp[i] = min(temp[i]+((ep/10)*delta_noise), 0)
    '''
    return temp
