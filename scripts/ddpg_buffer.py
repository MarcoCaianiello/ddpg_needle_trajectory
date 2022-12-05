#!/usr/bin/env python3

import tensorflow as tf
import numpy as np

import ddpg_param as prm
import ddpg_tool as tool


# The `Buffer` class implements the struct of the buffer, both Replay and Demonstration, for DDPG.

class Buffer:
    def __init__(self, buffer_capacity=1000, batch_size=64, asDemonstration=False):
        # Number of "experiences" to store at max
        self.buffer_capacity = buffer_capacity
        # Num of tuples to train on.
        self.batch_size = batch_size

        # Its tells us num of times record() was called.
        self.buffer_counter = 0

        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        self.state_buffer = np.zeros((self.buffer_capacity, prm.num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, prm.num_actions))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, prm.num_states))

        if asDemonstration:
            self.load_from_file()

    
    def load_from_file (self):
        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        # Loaded from a file where they are stored
        self.state_buffer = np.array(tool.read_from_file("state_buffer2.txt"))
        self.action_buffer = np.array(tool.read_from_file("action_buffer2.txt"))
        self.reward_buffer = np.array(tool.read_from_file("reward_buffer2.txt"))
        self.next_state_buffer = np.array(tool.read_from_file("next_state_buffer2.txt"))

        # Number of maximum saples provided
        self.buffer_capacity = len(self.reward_buffer)


    # Takes (s,a,r,s') obervation tuple as input
    # Store a new batch
    def record(self, obs_tuple):
        # Set index to zero if buffer_capacity is exceeded,
        # replacing old records
        index = self.buffer_counter % self.buffer_capacity
        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1]
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = obs_tuple[3]

        self.buffer_counter += 1

    # Return batch and Convert them to tensors
    def get_batch(self, batch_indices):

        state_batch = tf.convert_to_tensor(self.state_buffer[batch_indices])
        action_batch = tf.convert_to_tensor(self.action_buffer[batch_indices])
        reward_batch = tf.convert_to_tensor(self.reward_buffer[batch_indices])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(self.next_state_buffer[batch_indices])

        return state_batch, action_batch, reward_batch, next_state_batch
    
    def appendBuffer (self, buffer):
        for i in range(buffer.buffer_counter):
            self.record((buffer.state_buffer[i], buffer.action_buffer[i], buffer.reward_buffer[i], buffer.next_state_buffer[i]))
    
    def clearBuffer (self):
        self.state_buffer = np.zeros((self.buffer_capacity, prm.num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, prm.num_actions))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, prm.num_states))
        self.buffer_counter = 0




def get_index (replayBuffer, demBuffer, use_demonstration=False):
    
    prob=2
    batch_indices = 0
    dem_flag = 0

    if use_demonstration :
        prob = np.random.uniform(0, 1, 1)
    
    if (prob <= prm.eps_dem) and (demBuffer.buffer_counter>0) :
        # Get index from demonstration buffer
        batch_indices = np.random.choice(demBuffer.buffer_capacity, demBuffer.batch_size)
        dem_flag = 1
    else:
        # Get index from replay buffer
        record_range = min(replayBuffer.buffer_counter, replayBuffer.buffer_capacity)
        batch_indices = np.random.choice(record_range, replayBuffer.batch_size)


    return batch_indices, dem_flag

    