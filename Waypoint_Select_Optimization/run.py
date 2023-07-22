import os, psutil; print(psutil.Process(os.getpid()).memory_info().rss / 1024 ** 2)

#!/usr/bin/env python3
# run.py

#import statement
import DragonFlyFunctions as dff
import numpy as np

"""
Description: This script select the optimized waypoint as the node propagates throughout the misson, 
             while managing the MAVLINK data flow from Pixhawk Flight Controller.
Author: Kasidit Muenprasitivej, Evan Glenn, Aaron Riley
Date Created: May 23, 2023
Date Modified: July 14, 2023
Version: 1.0
Python Version: 3.9.7
Dependencies: ~
License: Purdue University
"""

#Choose Test Case Number
#opt = dff.getOption()

#Retrieve sounding data and target coordinates provided by NASA
opt_list = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10",
            "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
            "21", "22", "23", "24", "25", "26", "27", "28", "29", "30"]

for opt in opt_list:
    print("Test Case", opt)
    altitude, wind_velocity, waypoints = dff.getSoundingDataAndTargetCoord(opt)

    #Retrive predicted trajectory of the node
    predictedTrajectory = dff.getPredictedTrajectory()

    #Retrive the current GPS coordinate of node from Pixhawk Flight Controller via MAVLINK
    """ Here, we use a placeholder GPS coordinate to run the code"""
    initialPositionLatLongAlt = np.array([35.563751, -105.244449, 36576]) #Lat (deg), Long (deg), Atl (meters)

    #Simulate the node's trajectory to each targeted waypoint and select the best possible waypoint
    plotTraj = True
    printTraj = False
    waypointSuccesses = optimal_waypoint = dff.simulate(altitude, wind_velocity, predictedTrajectory, waypoints, initialPositionLatLongAlt, plotTraj, printTraj)
    dff.printResult(waypointSuccesses, opt)