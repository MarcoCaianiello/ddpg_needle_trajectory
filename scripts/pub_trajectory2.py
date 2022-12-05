#!/usr/bin/env python

import rospy
import math
import ddpg_tool as tool
import ddpg_param as prm
import numpy as np
import matplotlib.pyplot as plt
import pickle
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

class execute_path:
    
    def __init__(self):
        self.tipPosDes_publisher = rospy.Publisher("/dvrk/PSM2/tip_desired_path", Path, queue_size=1)
        self.jawPos_publisher = rospy.Publisher("/dvrk/PSM2/set_position_jaw", JointState, queue_size=1)
        self.joint_subscriber = rospy.Subscriber("/dvrk/Needle/tip_position_current_pose", Pose, self.tip_pose_cb)

        self.startPosition = np.zeros(3)
        self.startEuler = np.zeros(3)
        self.ready = 0

        self.main_loop()
    #---------------------------------------------------------------------
    

    def tip_pose_cb (self, msg):
        self.startPosition = [msg.position.x, msg.position.y, msg.position.z]
        self.startEuler = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.ready = 1
    #---------------------------------------------------------------------

    
    def get_path (self, start, goal, n):
        path=[]
        norm_path = []
        dist = goal - start
        k = n

        print(start)
        print(goal)

        path.append(start)
        norm_path.append(np.ones(len(dist)))
        
        for i in range(1,k):
            path.append(path[i-1]+dist/k)
            norm_path.append(norm_path[i-1]-(1/k))


        path.append(goal)
        norm_path.append(np.zeros(len(dist)))

        return path, norm_path
    #---------------------------------------------------------------------

    def send_path (self, path):

        msg = Path()
        msg.header.frame_id = "end_effector"
        msg.header.stamp = rospy.Time.now()

        for i in range(len(path)):
            for j in range(len(path[i])):
                path[i][j] = round(path[i][j], 4)
        
        for data in path:
            temp = PoseStamped()
            
            temp.pose.position.x = data[0]
            temp.pose.position.y = data[1]
            temp.pose.position.z = data[2]
            
            temp.pose.orientation.x = data[3]
            temp.pose.orientation.y = data[4]
            temp.pose.orientation.z = data[5]
            temp.pose.orientation.w = 0
            msg.poses.append(temp)
            print(data)

        while True:
            self.tipPosDes_publisher.publish(msg)
            rospy.sleep(2)
    #---------------------------------------------------------------------

    def close_jaw (self):
        temp = JointState()
        pos = -0.3
        temp.position.append(pos)
        self.jawPos_publisher.publish(temp)
    #---------------------------------------------------------------------

    def open_jaw (self):
        temp = JointState()
        pos = 0.6
        temp.position.append(pos)
        self.jawPos_publisher.publish(temp)
    #---------------------------------------------------------------------

    def save_demostration_buffer (self, start, goal):
        state_buffer = []
        action_buffer = []
        reward_buffer = []
        next_state_buffer = []

        n = [80, 100, 200, 500]
        for k in n:
            path, norm_path = self.get_path(start, goal, k)
            l1, l2, l3, l4 = self.get_demostration_buffer(norm_path)
            
            for i in range(k-1):
                state_buffer.append(l1[i])
                action_buffer.append(l2[i])
                reward_buffer.append(l3[i])
                next_state_buffer.append(l4[i])
        
        tool.save_on_file(state_buffer, "state_buffer2.txt")
        tool.save_on_file(action_buffer, "action_buffer2.txt")
        tool.save_on_file(reward_buffer, "reward_buffer2.txt")
        tool.save_on_file(next_state_buffer, "next_state_buffer2.txt")
    #---------------------------------------------------------------------
    
    def get_demostration_buffer(self, path):
        capacity = len(path)-1
        state_buffer = np.zeros((capacity, prm.num_states))
        action_buffer = np.zeros((capacity, prm.num_actions))
        reward_buffer = np.zeros((capacity, 1))
        next_state_buffer = np.zeros((capacity, prm.num_states))

        for i in range(capacity):
            state_buffer[i] = path[i]
            action_buffer[i] = path[i+1] - path[i]
            next_state_buffer[i] = path[i+1]

            p_prev = math.dist(path[capacity], state_buffer[i-1]) 
            p = math.dist(path[capacity], state_buffer[i])

            reward = 1/(1*math.pow(p,2))
            reward = (p<p_prev)*(4*math.pow(p_prev - p,2) + reward)
        
            reward_buffer[i] = reward
            print(action_buffer[i])

        return state_buffer, action_buffer, reward_buffer, next_state_buffer
    #---------------------------------------------------------------------

    def get_errors (self, path):
        pos = np.zeros((len(path), 3))
        euler = np.zeros((len(path), 3))
        ep = np.zeros((len(path),3))
        eo = np.zeros((len(path), 3))
        i=0
        for data in path:
            pos[i,:] = data[0:3]
            euler[i,:] = data[3:6]
            ep[i,:] = prm.goalPosition - pos[i,:]
            eo[i,:] = prm.goalEuler - euler[i,:]
            i=i+1
        #print("{}{}".format(pos,euler))


        print("Position Error: {}".format(ep[len(path)-1,:]))
        print("Abs Error: {}".format(math.dist(prm.goalPosition, pos[len(path)-1,:])))
        print("Orientation Error: {}".format(eo[len(path)-1,:]))

        return ep, eo
    #---------------------------------------------------------------------



    def main_loop(self):
        #while(not self.ready):
        #    rospy.sleep(0.1)
        
        #start = np.concatenate([prm.startPosition, prm.startEuler])
        #start = np.concatenate([self.startPosition, self.startEuler])
        start = np.concatenate([prm.startPosition, prm.startEuler])

        goal = np.concatenate([prm.goalPosition, prm.goalEuler])
        path, norm_path = self.get_path(start, goal, 97)
        tool.save_on_file(path, "opt_trajectory.txt")


        #self.get_errors(path)

        #path = np.array(tool.read_from_file("trajectory.txt"))

        
        #plt.plot(ep)
        #plt.xlabel("Position Error")
        #plt.ylabel("Time")
        #plt.show()

        '''
        while 1:
            self.close_jaw()
            rospy.sleep(2)

        with open("/home/marco/Desktop/ros_ws/src/dvrk_rl/dem_buffer/reward/rwd_ddpg_dem_07_5.txt", 'rb') as file_handler:
            data=pickle.load(file_handler)
        tool.plot_average_reward(data)
        '''

        print("MSG SEND")
        #self.save_demostration_buffer (start, goal)
        #self.send_path(path)
    #---------------------------------------------------------------------



if __name__ == '__main__':
    rospy.init_node('PUB_TRAJECTORY')
    execute_path()
    rospy.spin()

        