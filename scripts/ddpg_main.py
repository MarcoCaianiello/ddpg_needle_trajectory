#!/usr/bin/env python3

from re import A
from sre_constants import SUCCESS
import rospy
import time
import tensorflow as tnf
from tensorflow.keras import layers
import numpy as np
import math

import ddpg_param as prm
import ddpg_tool as tool
import ddpg_actor_critic_policy as NN
import ddpg_buffer as buffer

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Pose


class ddpg:

    def __init__(self):
        print("SERGIO")
        # ========== Topic Section ==========
        self.robot_subscriber = rospy.Subscriber("/dvrk/PSM1/current_state", String, self.robot_cb)
        self.joint_subscriber = rospy.Subscriber("/dvrk/Needle/tip_position_current_pose", Pose, self.tip_pose_cb)
        self.joint_publisher = rospy.Publisher("/dvrk/PSM1/set_position_goal_joint", JointState, queue_size=10)
        self.tipPosDes_publisher = rospy.Publisher("/dvrk/PSM1/tip_position_desired_pose", Pose, queue_size=1)
        self.clikFlag_subscriber = rospy.Subscriber("/dvrk/Needle/clik_flag", Int8, self.clikFlag_cb)
        self.jointIK_subscriber = rospy.Subscriber("/dvrk/PSM1/state_joint_IK", JointState, self.joint_cb)


        # ========== Joint Variable ==========
        self.position = None
        self.euler = None

        # ========== State Variable ==========
        self.start = np.concatenate([prm.startPosition, prm.startEuler])
        self.goal = np.concatenate([prm.goalPosition, prm.goalEuler])
        #self.dist = math.dist(prm.startPosition, prm.goalPosition)
        self.dist = np.absolute(self.goal - self.start)

        # ========== Normalization ==========
        self.goal_pos_state = prm.goalPosTh/(prm.goalPosition - prm.startPosition) 
        self.goal_orient_state = prm.goalOrientTh/(prm.goalEuler - prm.startEuler)

        #self.goal_pos_state = tool.divide(prm.goalPosTh, prm.goalPosition - prm.startPosition )
        #self.goal_orient_state = tool.divide(prm.goalOrientTh, prm.goalEuler - prm.startEuler )
        self.start_state = tool.divide(self.goal - self.start, self.dist)

        self.prev_state = np.absolute(self.start_state)
        self.goal_state = np.absolute(np.concatenate([self.goal_pos_state, self.goal_orient_state]))
        self.direction_buffer = tool.abs_buffer(self.start_state)

        '''
        print("Start State: " + tool.print_round_array(self.prev_state))
        print("Goal State: " + tool.print_round_array(self.goal_state))
        print("Goal Pos Th: " + tool.print_round_array(self.goal_pos_state))
        print("Goal Ort Th: " + tool.print_round_array(self.goal_orient_state))
        print("Direction Buffer: " + tool.print_round_array(self.direction_buffer))
        '''

        # ========== Initialization ==========
        self.tip_state = self.start
        self.prev_dist = self.dist
        self.min_dist = self.dist

        self.state = "READY"
        self.save_trajectory = True
        self.train_data = np.zeros(6)
        self.acc = 0
        self.temp_reward = 0
        self.best_reward = 0
        self.worst_reward = 10000
        self.temp_trajectory = []
        self.trajectory = []
        self.jointIK = []
        self.clikFlag = 0
        self.useOUA_Noise = True
        self.useDemonstration = False

        # ========== Neural Network Section ==========
        self.net = NN.DDPG_Neural_Network()

        # ========== Buffer Section ==========
        self.replayBuffer = buffer.Buffer(prm.buffer_capacity, prm.batch_size)
        self.demBuffer = buffer.Buffer(asDemonstration=True)
        self.tempBuffer = buffer.Buffer(prm.buffer_capacity, prm.batch_size)
        self.succBuffer = buffer.Buffer(prm.buffer_capacity, prm.batch_size)

        self.main_loop()
    #---------------------------------------------------------------------
    
    
    def robot_cb(self, msg):
        self.state = msg.data
    #---------------------------------------------------------------------

    def tip_pose_cb(self, msg):
        self.position = [msg.position.x, msg.position.y, msg.position.z]
        self.euler = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        #print("position {}".format(self.position))
        #print("euler {}".format(self.euler))
    #---------------------------------------------------------------------

    def clikFlag_cb(self, msg):
        self.clikFlag = msg.data
    #---------------------------------------------------------------------

    def joint_cb (self, msg):
        self.jointIK = msg.position
    #---------------------------------------------------------------------

    def reward_function (self, state, prev_state, joint, action_counter, reward=0, done=False, info=""):
        statePosition = [state[0], state[1], state[2]]
        statePosition_prev = [prev_state[0], prev_state[1], prev_state[2]]
        p_prev = math.dist(np.zeros(6), prev_state) #math.dist(self.goal_pos_state, statePosition_prev)
        p = math.dist(np.zeros(6), state) #math.dist(self.goal_pos_state, statePosition)
        goal_bool = True

        #Joint Section
        for i in range(6):

            if (joint[i]<=prm.q_min[i]):
                info = info + " - joint " + str(i+1) + " MIN"
                reward = -1
                done = True

            elif (joint[i]>=prm.q_max[i]):
                info = info + " - joint " + str(i+1) + " MAX"
                reward = -1
                done = True

        reward = (prm.max_actions - action_counter)/(100*math.pow(p,2))
        #reward = (p<p_prev)(5*math.pow(p_prev - p,2)) + reward #(p<p_prev)*(4*math.pow(p_prev - p,2) + reward)
        #print("Reward: {:.3f} ---- p: {:.3f}".format(reward, p))

        #Out of the workspace
        if (p>prm.outTh):
            reward = -100*math.sqrt(prm.num_states)
            info = "OUT OF BOUND"
            done = True

        #Find the goal
        for i in range(3):
            goal_bool *=  abs(state[i]) <= self.goal_state[i]
            
        if (goal_bool):
            reward = 1000
            info = "GOAL REACHED"
            done = True

        return reward, info, done
    #---------------------------------------------------------------------

    def main_loop(self):
        
        period = 0.05
        f = 1/period
        rate = rospy.Rate(f)

        while (self.state != 'READY'):
            rospy.sleep(period/3)

        if self.save_trajectory:
            self.net = tool.load_weights(self.net)
            prm.total_episodes = 100

        for ep in range(prm.total_episodes):

            #print(self.prev_state)


            prev_state = self.prev_state
            prev_tip_state = self.tip_state
            episodic_reward = 0
            action_counter = 0
            reward = 0
            info = ""
            self.temp_trajectory.clear()
            start_time = time.time()
            
            while True:
               
                tf_prev_state = tnf.expand_dims(tnf.convert_to_tensor(prev_state), 0)
                
                # ========== Action Section ==========
                action = self.net.getActions(tf_prev_state, tool.noise_object(tool.ou_noise, ep, info, self.useOUA_Noise))
                action_counter += 1

                action_ph = np.array(np.squeeze(action[0]))
                #print("ACTION -> "+ tool.print_round_array(action_ph))

                state = np.zeros((prm.num_states))
                tip_state = np.zeros((prm.num_states))
                done = False
                
                for i in range(prm.num_states):
                    state[i] = prev_state[i] + action_ph[i]
                    tip_state[i] = prev_tip_state[i] - (action_ph[i]*self.dist[i]*self.direction_buffer[i])

                # ========== Send Position to Robot ==========
                
                self.jointIK = np.ones((6))*0.2
                '''
                #self.jointIK = []

                while ((self.clikFlag == 0) or (self.jointIK == [])):
                    #self.tipPosDes_publisher.publish(tool.convert_to_PoseMsg(state))
                    self.tipPosDes_publisher.publish(tool.convert_to_PoseMsg(tip_state))
                    #print(self.jointIK)
                    #print(self.clikFlag)
                    rospy.sleep(0.05)
                '''    

                self.clikFlag = 0
                #print(action_counter)
                #print(state)

                # ========== Reward Section ==========
                if (action_counter < prm.max_actions):

                    reward, info, done = self.reward_function(state, prev_state, self.jointIK, action_counter)
                    end_time = time.time()

                else:
                    reward -= 5*prm.max_actions
                    info = "MAX ACTION"
                    done = True

                episodic_reward += reward

                # ========== Learning Section ==========
                self.replayBuffer.record((prev_state, np.array(action), reward, state))
                self.net.learn(self.replayBuffer, self.demBuffer, self.useDemonstration)
                '''
                self.tempBuffer.record((prev_state, np.array(action), reward, state))
                if ep==0:
                    self.net.learn(self.tempBuffer, self.succBuffer, False)
                else:
                    self.net.learn(self.replayBuffer, self.succBuffer, self.useDemonstration)
                '''
                

                
                prev_state = state
                prev_tip_state = tip_state

                if (self.save_trajectory):
                    self.temp_trajectory.append(tip_state)


                self.joint_publisher.publish(tool.convert_to_JointStateMsg(self.jointIK))

                # End this episode when `done` is True
                if done:
                    break
                
                rate.sleep()
                

                #self.joint_publisher.publish(pos_msg)
                #print("SERGIO UP")
                #rate.sleep()
            #---------------------------------------------------------------------


            if (info == "GOAL REACHED") and (ep != 0):
                self.succBuffer.appendBuffer(self.tempBuffer)
            else:
                self.replayBuffer.appendBuffer(self.tempBuffer)

            self.tempBuffer.clearBuffer()

            prm.ep_reward_list.append(episodic_reward)
            avg_reward = tool.mean_each_x_episodes(prm.ep_reward_list, 1)
            #print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
            print("Episode * {} * Ep Reward is ==> {:.3f}".format(ep, episodic_reward))
            print("End State: " + tool.print_round_array(state))
            print("Epsilon Dem: {}".format(prm.eps_dem))
            print("Execution Time: {}".format(end_time-start_time))
            print("End Condition: {}".format(info))
            print("Dist: " + tool.print_round_array(np.absolute(state - self.start)))#(self.dist))
            print("Goal Condition: " + tool.print_round_array(self.goal_state))
            print("------------------------------------------------------------")


            if (self.save_trajectory):
                if (info == "GOAL REACHED"):
                    self.acc += 1 
                    if (episodic_reward > self.best_reward): 
                        self.trajectory = self.temp_trajectory.copy()
                        self.best_reward = episodic_reward
                        self.train_data [0] = end_time-start_time
                        self.train_data [2] = action_counter
                        self.train_data [4] = self.best_reward
                    elif (episodic_reward < self.worst_reward):
                        self.worst_reward = episodic_reward
                        self.train_data [1] = end_time-start_time
                        self.train_data [3] = action_counter
                        self.train_data [5] = self.worst_reward

            prm.avg_reward_list.append(avg_reward)
            #prm.eps_dem = min(prm.eps_dem + prm.delta_dem, 1)

        if (self.save_trajectory):
            tool.save_on_file(self.trajectory, "trajectory.txt")
            tool.save_on_file(self.train_data, "train_data.txt")
            print("SERGIOVED")
            print(self.acc)
        
        tool.save_on_file(prm.avg_reward_list, "rwd_ddpg.txt")
        tool.save_weights(self.net)
        tool.plot_average_reward (prm.avg_reward_list)
#---------------------------------------------------------------------
        

if __name__ == '__main__':
    rospy.init_node('DVRK_DDPG')
    ddpg()
    rospy.spin()
#---------------------------------------------------------------------