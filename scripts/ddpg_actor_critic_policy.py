#!/usr/bin/env python3

import tensorflow as tf
from tensorflow.keras import layers
from tensorflow.keras.initializers import glorot_normal
import numpy as np
import ddpg_tool as tool
import ddpg_param as prm
import ddpg_buffer as buffer


# Actor Neural Network
# Get as input the state and gerate the actions
def actor_network():
    # Initialize weights by normal distribution
    last_init = tf.random_normal_initializer(stddev=0.001)

    inputs = layers.Input(shape=(prm.num_states,))
    out = layers.Dense(600, activation=tf.nn.leaky_relu, kernel_initializer=glorot_normal())(inputs)
    out = layers.Dense(300, activation=tf.nn.leaky_relu, kernel_initializer=glorot_normal())(out)
    outputs = layers.Dense(prm.num_actions, activation="tanh", kernel_initializer=last_init)(out)
    outputs = outputs# * prm.max_distance

    model = tf.keras.Model(inputs, outputs)
    return model


# Critic Neural Network
# Get as input the state and action and gerate the value
def critic_network():

    last_init = tf.random_normal_initializer(stddev=0.0001)

    # State as input
    state_input = layers.Input(shape=(prm.num_states,))
    state_out = layers.Dense(600, activation=tf.nn.leaky_relu, kernel_initializer=glorot_normal())(state_input)
    state_out = layers.Dense(300, activation=tf.nn.leaky_relu, kernel_initializer=glorot_normal())(state_out)

    # Action as input
    action_input = layers.Input(shape=(prm.num_actions,))
    action_out = layers.Dense(300, activation=tf.nn.leaky_relu, kernel_initializer=glorot_normal())(action_input/prm.max_actions) #(action_input)

    # Both are passed through seperate layer before concatenating
    added = tf.keras.layers.Add()([state_out, action_out])

    added = tf.keras.layers.BatchNormalization()(added)
    out = layers.Dense(300, activation=tf.nn.leaky_relu, kernel_initializer=glorot_normal())(added)
    outs = tf.keras.layers.BatchNormalization()(out)
    outputs = tf.keras.layers.Dense(1, kernel_initializer=last_init)(outs)

    # Outputs single value for give state-action
    model = tf.keras.Model([state_input, action_input], outputs)

    return model


class DDPG_Neural_Network:
    def __init__(self):
        # Initialize standard and target networks
        self.actor_model = actor_network()
        self.critic_model = critic_network()

        self.target_actor = actor_network()
        self.target_critic = critic_network()

        # Making the weights equal initially
        self.target_actor.set_weights(self.actor_model.get_weights())
        self.target_critic.set_weights(self.critic_model.get_weights())

        # Define Optimizer
        self.critic_optimizer = tf.keras.optimizers.Adam(prm.critic_lr)
        self.actor_optimizer = tf.keras.optimizers.Adam(prm.actor_lr)

    # Return the action from the Actor NN
    # affected by the exloration noise
    def getActions(self, state, noise):

        actions = self.actor_model(state)
        #print(actions.numpy())

        # Adding noise to action for exploration
        
        #if ((noise%2)==0): noise = -1*noise
        exploration_actions = actions.numpy() + noise

        # Check in the action is in the bounds
        legal_action = np.clip(exploration_actions, prm.min_distance, prm.max_distance)

        return [legal_action]

    # We compute the loss and update parameters
    def learn(self, replayBuffer, demBuffer, use_demonstration=False):
        # Randomly sample indices
        batch_indices, dem_flag = buffer.get_index(replayBuffer, demBuffer, use_demonstration)

        if (dem_flag == 1):
            state_batch, action_batch, reward_batch, next_state_batch = demBuffer.get_batch(batch_indices)
        else:    
            state_batch, action_batch, reward_batch, next_state_batch = replayBuffer.get_batch(batch_indices)
        
        self.update(state_batch, action_batch, reward_batch, next_state_batch)
        self.update_target(self.target_actor.variables, self.actor_model.variables, prm.tau)
        self.update_target(self.target_critic.variables, self.critic_model.variables, prm.tau)

    # Decorating with tf.function allows TensorFlow to build a static graph out of the logic and computations in our function.
    # This provides a large speed up for blocks of code that contain many small TensorFlow operations such as this one.
    @tf.function
    def update(self, state_batch, action_batch, reward_batch, next_state_batch):
        
        # Training and updating Actor & Critic networks.
        with tf.GradientTape() as tape:
            target_actions = self.target_actor(next_state_batch, training=True)
            y = reward_batch + prm.gamma * self.target_critic([next_state_batch, target_actions], training=True)
            
            critic_value = self.critic_model([state_batch, action_batch], training=True)
            critic_loss =  tf.math.reduce_mean(tf.math.abs(y - critic_value))

        critic_grad = tape.gradient(critic_loss, self.critic_model.trainable_variables)
        self.critic_optimizer.apply_gradients(zip(critic_grad, self.critic_model.trainable_variables))

        with tf.GradientTape() as tape:
            actions = self.actor_model(state_batch, training=True)
            critic_value = self.critic_model([state_batch, actions], training=True)
            # Used `-value` as we want to maximize the value given
            # by the critic for our actions
            actor_loss = tf.math.reduce_mean(critic_value) #-tf.math.reduce_mean(critic_value)

        actor_grad = tape.gradient(actor_loss, self.actor_model.trainable_variables)
        self.actor_optimizer.apply_gradients(zip(actor_grad, self.actor_model.trainable_variables))
        

    # This update target parameters slowly
    # Based on rate `tau`, which is much less than one.
    @tf.function
    def update_target(self, target_weights, weights, tau=prm.tau):
        for (a, b) in zip(target_weights, weights):
            a.assign(b * tau + a * (1 - tau))



