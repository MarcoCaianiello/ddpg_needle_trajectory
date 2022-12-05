#!/usr/bin/env python

import rospy
import math
import ddpg_tool as tool
import ddpg_param as prm
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState



def get_demostration_buffer(path):
    capacity = len(path)-1
    state_buffer = np.zeros((capacity, prm.num_states))
    action_buffer = np.zeros((capacity, prm.num_actions))
    reward_buffer = np.zeros((capacity, 1))
    next_state_buffer = np.zeros((capacity, prm.num_states))

    for i in range(capacity):
        state_buffer[i] = path[i]
        action_buffer[i] = path[i+1] - path[i]
        next_state_buffer[i] = path[i+1]
    
    reward_buffer[capacity-1] = 1

    '''
    print(state_buffer[capacity-1])
    print(action_buffer[capacity-1])
    print(reward_buffer[capacity-1])
    print(next_state_buffer[capacity-1])

    print(path[len(path)-1])
    '''

    return state_buffer, action_buffer, reward_buffer, next_state_buffer

def save_demostration_buffer (start, goal):
    state_buffer = []
    action_buffer = []
    reward_buffer = []
    next_state_buffer = []

    n = [30, 50, 80, 100]
    for k in n:
        path, norm_path = get_path(start, goal, k)
        l1, l2, l3, l4 = get_demostration_buffer(norm_path)
        print(l3)
        for i in range(k-1):
            state_buffer.append(l1[i])
            action_buffer.append(l2[i])
            reward_buffer.append(l3[i])
            next_state_buffer.append(l4[i])
    
    tool.save_on_file(state_buffer, "state_buffer.txt")
    tool.save_on_file(action_buffer, "action_buffer.txt")
    tool.save_on_file(reward_buffer, "reward_buffer.txt")
    tool.save_on_file(next_state_buffer, "next_state_buffer.txt")

def tip_pose_cb (msg):
        startPosition = [msg.position.x, msg.position.y, msg.position.z]
        startEuler = [msg.orientation.x, msg.orientation.y, msg.orientation.z]

class execute_path:
    
    def __init__(self):
        self.tipPosDes_publisher = rospy.Publisher("/dvrk/PSM2/tip_desired_path", Path, queue_size=1)
        self.jawPos_publisher = rospy.Publisher("/dvrk/PSM2/set_position_jaw", JointState, queue_size=1)
        self.joint_subscriber = rospy.Subscriber("/dvrk/Needle/tip_position_current_pose", Pose, self.tip_pose_cb)

        self.startPosition = np.zeros(3)
        self.startEuler = np.zeros(3)
        self.ready = 0

        self.main_loop()

    def tip_pose_cb (self, msg):
        self.startPosition = [msg.position.x, msg.position.y, msg.position.z]
        self.startEuler = [msg.orientation.x, msg.orientation.y, msg.orientation.z]
        self.ready = 1

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

    def get_msg (self, path):

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
        '''
        print(path[0])
        print(path[len(path)-1])
        '''
        while True:
            self.tipPosDes_publisher.publish(msg)
            rospy.sleep(2)
        
    def close_jaw (self):
        temp = JointState()
        pos = -0.3
        temp.position.append(pos)
        self.jawPos_publisher.publish(temp)

    def open_jaw (self):
        temp = JointState()
        pos = 0.6
        temp.position.append(pos)
        self.jawPos_publisher.publish(temp)
        
    def main_loop(self):
        while(not self.ready):
            rospy.sleep(0.1)
        
        #start = np.concatenate([prm.startPosition, prm.startEuler])
        start = np.concatenate([self.startPosition, self.startEuler])
        goal = np.concatenate([prm.startPosition, prm.startEuler])
        #goal = np.concatenate([prm.goalPosition, prm.goalEuler])
        path, norm_path = self.get_path(start, goal, 50)
        #path = self.rotate(start)

        #for data in path: print(data)

        #path = np.array(tool.read_from_file("trajectory.txt"))
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
        
        #plt.plot(ep)
        #plt.xlabel("Position Error")
        #plt.ylabel("Time")
        #plt.show()

        '''
        while 1:
            self.close_jaw()
            rospy.sleep(2)
        '''
        print("MSG SEND")
        #save_demostration_buffer (start, goal)
        self.get_msg(path)



    

if __name__ == '__main__':
    rospy.init_node('PUB_TRAJECTORY')
    execute_path()
    rospy.spin()

    

