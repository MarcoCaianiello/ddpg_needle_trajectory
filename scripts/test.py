#!/usr/bin/env python

import math as m
import numpy as np
import math
import ddpg_tool as tool
import ddpg_param as prm
import pickle
import scipy.io

if __name__ == '__main__':

    filename = "trajectory_dem_04_1"
    path = prm.rwd_path + "mat/"
    
    with open(path + filename + ".txt", 'rb') as file_handler:
        data=pickle.load(file_handler)
    tool.plot_average_reward(data)
    #print(data)

    scipy.io.savemat(path + filename + ".mat", mdict={filename: data})
        

    

