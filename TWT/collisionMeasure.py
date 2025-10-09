# This code is for batch processing ns3 trace and PHY state logs for PSM with uplink TCP traffic
# Author : Shyam Krishnan Venkateswaran
# Email: shyam1@gatech.edu
# Variables to take care of before each run will be tagged with #changeThisBeforeRun

#  This script counts collisions - needs only one log file - TX states


# %%

import numpy as np
import matplotlib.pyplot as plt
import csv
from matplotlib.pyplot import figure
import json
import random
import glob     # for parsing directories
import os       # for clearing terminal in windows OS
import pandas as pd
from pathlib import Path       # to handle directories across OSs
from random import randrange



# %%

# File management


# naming convention for log files from ns3
# stateLogs for PSM STA must be named as 'log######.statelog', where ###### are 6 numbers 0-9
# Ascii traces  must be named as 'log######.asciitr', where ###### are 6 numbers 0-9
# Tx traces (have only Tx PHY trace for all WiFi nodes) must be named as 'log######.txlog', where ###### are 6 numbers 0-9
# The '#####' will be used as IDs to find corresponding log - trace pairs. 

# Output CSV file name

csvFileName = 'Feb15_50StaCollision.csv'          #changeThisBeforeRun
StaNodeDeviceStringASCII = "NodeList/1/DeviceList/0"           # note: Make sure this is the STA node and interface of interest from ns3. Other network events will be ignored.
ApNodeDeviceStringASCII = "NodeList/0/DeviceList/1"           # note: Make sure this is the AP node and interface of interest from ns3. Other network events will be ignored.

# Following lists will be appended at the end of each iteration. The lists are ordered and correspond to each other in the correct order.
index_list = [] 
totalDuration_us_list = []   # Any time awake that is not in Tx is considered Rx
collisionCount_list = []
totalDuration_us_list = []


data_folder = Path("MU_logs/")                                             

ns3_txstate_log_list_temp = list(data_folder.glob("**/*.txlog"))
ns3_txstate_log_list = [str(x) for x in ns3_txstate_log_list_temp]


stateLogStartTime_us = 10 * 1e6          # PHY state log will be imported only after this time from beginning
stateLogEndTime_us = 100 * 1e6          # PHY state log will be imported only till this time

ns3_txstate_log_list.sort()


logCount = len(ns3_txstate_log_list)
print (f"Total log count: {logCount}")


print(f"Number of logs to be processed: {logCount}")
# -----------------------------------------------------------
# -----------------------------------------------------------



# %%
# ----------------------------------------------------------------------------------------------------10380
# Master Loop starts here


for logFilesCounter in range(logCount):
    

    ns3_txstate_log = ns3_txstate_log_list[logFilesCounter]

    # Checking if log and trace files have the same number
    print(f"ns3_txstate_log:{ns3_txstate_log}")
    
    index_list.append(ns3_txstate_log.split('.')[0][-6:])
    print(f"\nProcessing log file ({logFilesCounter+1}/{logCount})")
    
    

    
       
    # %%

    # Channel Busy time and ratio calculation

    # Importing and parsing Tx only combined State log as a dict
    # Note - if there are multiple Tx states starting at same time, the longest tx duration will be taken
    totalCollisions = 0
    totalDuration_ns = 0
    totalStartTime_ns = 0
    totalEndTime_ns = 0
    prevTxEnd_ns = 0
    currentTxStart_ns = 0
    currentTxEnd_ns = 0 




    # full_tx_only_duration_us_Dict = dict()          # Dict keys are in ns; Dict values are is us
    with open(ns3_txstate_log) as file1:
        Lines = file1.readlines() 
        for line in Lines: 
            timeStamp_ns = line.split(";")[0]
            timeStamp_ns = int(timeStamp_ns.split("=")[1])
            currentTxStart_ns = timeStamp_ns
            if (currentTxStart_ns > stateLogEndTime_us*1000):
                break
            if (currentTxStart_ns < stateLogStartTime_us * 1000 ) or (currentTxStart_ns > stateLogEndTime_us*1000 ) :
                continue
            if totalStartTime_ns == 0:
                totalStartTime_ns = currentTxStart_ns
            
            
            txDuration_ns = line.split(";")[1]
            currentTxEnd_ns = currentTxStart_ns +  int(txDuration_ns.split("=")[1])
            if totalEndTime_ns < currentTxStart_ns:
                totalEndTime_ns = currentTxEnd_ns
            if (currentTxStart_ns < prevTxEnd_ns):
                totalCollisions += 1
                if (currentTxEnd_ns > prevTxEnd_ns):
                    prevTxEnd_ns = currentTxEnd_ns
            else:
                prevTxEnd_ns = currentTxEnd_ns

    totalDuration_ns = totalEndTime_ns - totalStartTime_ns
    totalDuration_us = int(totalDuration_ns/1000)
    collisionCount_this = totalCollisions
    print(f"Collisions: {collisionCount_this}")
    print(f"totalDuration_us: {totalDuration_us}")

    collisionCount_list.append (collisionCount_this)
    totalDuration_us_list.append (totalDuration_us)

    # Master Loop ends here
    # ----------------------------------------------------------------------------------------------------
# os.system('cls||clear')

# %%
print(f"\n\n----------------------------------------------------------------\nCompleted processing {logCount} simulations.")

# print(f"Average current List in mA: {averageCurrent_mA_list}")

# writing to a csv file
# CSV file format
#  averageCurrent_mA

dataFrameToWrite = pd.DataFrame(list(zip(index_list, totalDuration_us_list, collisionCount_list)), columns= ['SimulationID', 'totalDuration_us_list', 'collisionCount_list'])
with pd.option_context('display.max_rows', None, 'display.max_columns', None):
    print(dataFrameToWrite)

dataFrameToWrite.to_csv(csvFileName, index  = False)



