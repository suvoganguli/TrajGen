import numpy as np
import matplotlib.pyplot as plt
import os
import glob
import printPlots
from utils import *

oldpwd = os.getcwd()

dirTopName = './run_2018-06-07_batch'
os.chdir(dirTopName)

listDir = glob.glob("run*")

for dirName in listDir:
    os.chdir(dirName)
    dummy = glob.glob("log*.txt")
    fileName = dummy[0]

    f = file(fileName, 'r')
    cols, indexToName = getColumns(f, delim=" ", header=False)

    cpuTime = np.array(cols[10]).astype(np.float)

    if np.max(cpuTime) > 60:   # and np.argmax(cpuTime) < 20:
        print(dirName)
        #plt.plot(cpuTime[:])
        None
    else:
        plt.plot(cpuTime[:])
        #plt.pause(0.1)
        None

    plt.xlabel('MPC Steps')
    plt.ylabel('CPU Time [sec]')
    plt.ylim([0,60])

    os.chdir('../')

plt.show()

