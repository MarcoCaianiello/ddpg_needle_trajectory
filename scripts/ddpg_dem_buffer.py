#!/usr/bin/env python3

import tensorflow as tf
import numpy as np

import ddpg_param as prm
import ddpg_tool as tool

"""
The `Dem_Buffer` class contain buffers 
with state, action and reward from demostration.

"""

class Dem_Buffer:
    def __init__(self):
        
        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        # Loaded from a file where they are stored
        self.state_buffer = np.array(tool.read_from_file("state_buffer.txt"))
        self.action_buffer = np.array(tool.read_from_file("action_buffer.txt"))
        self.reward_buffer = np.array(tool.read_from_file("reward_buffer.txt"))
        self.next_state_buffer = np.array(tool.read_from_file("next_state_buffer.txt"))

        # Number of maximum saples provided
        self.buffer_capacity = len(self.reward_buffer)
        print(self.buffer_capacity)
    
    def get_batch(self, index):

        state_batch = tf.convert_to_tensor(self.state_buffer[index])
        action_batch = tf.convert_to_tensor(self.action_buffer[index])
        reward_batch = tf.convert_to_tensor(self.reward_buffer[index])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(self.next_state_buffer[index])

        return state_batch, action_batch, reward_batch, next_state_batch

    