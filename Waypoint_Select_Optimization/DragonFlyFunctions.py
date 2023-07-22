
#!/usr/bin/env python3
# DragonFlyFunctions.py

#import statements
import numpy as np
import matplotlib.pyplot as plt

"""
Description: This script contains all user defined functions used in run.py
Author: Kasidit Muenprasitivej, Evan Glenn, Aaron Riley
Date Created: May 23, 2023
Date Modified: July 14, 2023
Version: 1.0
Python Version: 3.9.7
Dependencies: ~
License: Purdue University
"""

#DEFINE CONSTANTS
#Unit conversion
ft_to_m = 0.3048
kts_to_m_s = 0.514


def getOption():
    """
    getOption - ask user for an option number to determine which test case to run

    :return: option number indicating the test case number
    """ 
    opt_list = ["01", "02", "03", "04", "05", "06", "07", "08", "09", "10",
            "11", "12", "13", "14", "15", "16", "17", "18", "19", "20",
            "21", "22", "23", "24", "25", "26", "27", "28", "29", "30"]

    valid_opt = False
    while(not valid_opt):
        opt = str(input("Choose the test case by entering two digits number (from 01 - 30): "))
        if opt in opt_list:
            valid_opt = True
        else:
            print("Invalid test case. Try again.")
    return opt



def getClosest(altitudeOfInterest, altitude_array, arrayOfInterest):
    """
    getClosest - retrive the closest values from sounding data/predicted data at the altitude of interest

    :param altitudeOfInterest: Altitude of interest [meter]
    :param altitude_array: Array of altitudes with corresponding defined data
    :param arrayOfInterest: Array of the corresponding data
    :return: Closest data point in the Array of the corresponding data at given altitude of interest
    """ 
    altitude_array = np.asarray(altitude_array)
    idx = (np.abs(altitude_array - altitudeOfInterest)).argmin()
    return arrayOfInterest[idx]

def getSoundingDataAndTargetCoord(opt):
    """
    getSoundingDataAndTargetCoord - read and return all sounding data and target coordinate as valued arrays, given the option digit

    :param opt: option digit ranging from 01 - 30
    :return: altitude_array, wind_velocity_array, waypoints_array (i.e., 6 target points)
    """ 

    #Initialize lists
    altitude=[]
    wind_direction=[]
    wind_speed=[]

    #read sounding file and extract data
    with open ("soundAndTarg/soundings/predict_" + opt + "_sound.txt", "r") as f:
        next(f)
        winds_aloft = f.read().split('\n')
        for i in winds_aloft:
            array = i.split("\t")
            #Handling NaN value in dataset
            if len(array) < 5:
                continue
            altitude.append(float(array[1]))
            wind_direction.append(float(array[2]))
            wind_speed.append(float(array[3]))

    altitude = np.array(altitude) * ft_to_m #m
    wind_direction = np.array(wind_direction) #deg
    wind_speed = np.array(wind_speed) * kts_to_m_s #m/s

    #----------------------------

    #Initialize lists
    latitude = []
    longitude = []

    #read target coordinate and extract data
    with open ("soundAndTarg/targets/predict_" +  opt + "_targets.txt", "r") as f:
        next(f)
        waypointsRaw = f.read().split('\n')
        for i in waypointsRaw:
            array = i.split(",")
            #Handling NaN value in dataset
            if len(array) < 4:
                continue
            latitude.append(float(array[2]))
            longitude.append(float(array[1]))

    latitude = np.array(latitude)
    longitude = np.array(longitude)

    waypoints = []
    for i in range(len(latitude)):
        waypoints.append(np.array([latitude[i], longitude[i],0]))
    waypoints = np.array(waypoints)

    #----------------------------

    #create wind velocity vector
    v_list = []
    for i in range(len(wind_speed)):
        vx = wind_speed[i] * np.sin(wind_direction[i] * np.pi/180)
        vy = wind_speed[i] * np.cos(wind_direction[i] * np.pi/180)
        v = np.array([vx, vy, 0])
        v_list.append(v)
    wind_velocity = np.array(v_list)

    return altitude, wind_velocity, waypoints



def getPredictedTrajectory():
    """
    getPredictedTrajectory - read and return predicted trajectory of the node provided from descentProfileCallScript.m (MATLAB) 

    :return: predicted trajectory of the node
    """ 

    #Initialize lists
    predictedAltitude = []
    verticalVelocity = []
    horizontalVelocity = []
    predictedTrajectory = []

    #read predicted trajectory and extract data
    with open ("descentTrajectoryPrediction/predictedTrajectory.dat", "r") as f:
        next(f)
        predictedTrajectoryRaw = f.read().split('\n')
        for i in predictedTrajectoryRaw:
            array = i.split(",")
            #Handling NaN value in dataset
            if len(array) < 6:
                continue
            predictedAltitude.append(float(array[0]))
            verticalVelocity.append(float(array[1]))
            horizontalVelocity.append(float(array[3]))

        predictedAltitude = np.array(predictedAltitude)
        verticalVelocity = np.array(verticalVelocity)
        horizontalVelocity = np.array(horizontalVelocity)

    for i in range(len(predictedAltitude)):
        predictedTrajectory.append(np.array([predictedAltitude[i], verticalVelocity[i], horizontalVelocity[i]]))
    predictedTrajectory = np.array(predictedTrajectory)

    return predictedTrajectory



def simulate(altitude, wind_velocity, predictedTrajectory, waypoints, initialPositionLatLongAlt, plotTraj, printTraj):
    """
    simulate - Simulate the node's trajectory (from current position) to each targeted waypoint
               in order to determine the best waypoint to navigate.

    :param altitude: Array of altitudes
    :param wind_velocity: Array of wind velocity vectors corresponding to each altitude in the altitude array
    :param predictedTrajectory: Predicted trajectory of the node including vertical velocity (z) and resultant horizontal velocity (x + y)
    :param waypoints: Array of coordinates of the targeted waypoint (A - F)
    :param initialPositionLatLongAlt: current positon of the node given from the Pixhawk Flight Controller's GPS data
    :param plotTraj: Boolean variable to show/hide the plots of predicted trajectories
    :return: describe what it returns
    """ 
    earthRadius = 6378 * 1000 #m
    timeStep = 1 #s
    margin = 250 #m
    groundLevel = 1223 #m
    nFig = 0

    predictedAltitude = predictedTrajectory[:, 0]
    verticalVelocity = predictedTrajectory[:, 1]
    horizontalVelocity = predictedTrajectory[:, 2]
    position = np.array([0, 0, 0])
    targetDirection = np.array([0, 0, 0])

    waypointSuccesses = []
    
    waypointIdx = 0
    for waypoint in waypoints:
    
        positionLatLongAlt = initialPositionLatLongAlt

        #convert coordinate [lat, long, m] to xyz [m, m, m]
        position[0] = (positionLatLongAlt[1] - waypoint[1]) * earthRadius * np.pi / 180
        position[1] = (positionLatLongAlt[0] - waypoint[0]) * earthRadius * np.pi / 180
        position[2] = positionLatLongAlt[2]
        
        #Find the directional vector toward the waypoint
        targetDirection[0] = (positionLatLongAlt[1] - waypoint[1]) * earthRadius * np.pi / 180
        targetDirection[1] = (positionLatLongAlt[0] - waypoint[0]) * earthRadius * np.pi / 180
        targetDirection[2] = 0 #-positionLatLongAlt[2]
        targetDirectionUnit = targetDirection / np.linalg.norm(targetDirection)

        #Define list variable to keep track of the node's simulated position history
        xPosition = []
        yPosition = []
        zPosition = []

        soundingArray = []
        soundingArrayX = []
        soundingArrayY = []
        xPositionSounding = []
        yPositionSounding = []
        zPositionSounding = []

        #Begin Trajectory Prediction
        i = 0
        while position[2] > (groundLevel):
            i = i + 1
            currentAltitude = position[2]

            #Get closest wind velocity at current altitude
            soundingVelocity = getClosest(currentAltitude, altitude, wind_velocity)
            
            if (i % 17 == 0):
                soundingArrayX.append(soundingVelocity[0])
                soundingArrayY.append(soundingVelocity[1]) 
                xPositionSounding.append(position[0])
                yPositionSounding.append(position[1])
                zPositionSounding.append(position[2])


            #Get the node speed, predicted at the current altitude
            predictedHorizontalVelocity = getClosest(currentAltitude, predictedAltitude, horizontalVelocity) # sqrt(Vx^2 + Vy^2)
            predictedVerticalVelocity = getClosest(currentAltitude, predictedAltitude, verticalVelocity) #Vz

            #Get the node velocity vector, predicted at the current altitude
            predictedVelocityDirected = [targetDirectionUnit[0] * predictedHorizontalVelocity, targetDirectionUnit[1] * predictedHorizontalVelocity, predictedVerticalVelocity]
            
            #Vector addition 
            trueVelocity = soundingVelocity + predictedVelocityDirected

            position = position + trueVelocity * timeStep
            xPosition.append(position[0])
            yPosition.append(position[1])
            zPosition.append(position[2])

            distanceToTarget = np.linalg.norm(position)
            targetDirectionUnit[0] = (-position [0]) / distanceToTarget
            targetDirectionUnit[1] = (-position [1]) / distanceToTarget

            if printTraj:
                print("distance to Targ", np.linalg.norm(position[0:2]))
            
            reachTarg = []
            if (np.linalg.norm(position[0:2]) <= margin):
                #send waypoint to Pixhawk
                waypointSuccesses.append([waypointIdx, position[2]])
                reachTarg = position

                if printTraj:
                    print("Reached!", reachTarg)
                break

        waypointIdx += 1
            
        if plotTraj:
            nFig += 1
            fig = plt.figure(nFig)
            ax = plt.axes(projection='3d')
            if(len(reachTarg) == 3):
                plt.plot(reachTarg[0], reachTarg[1],reachTarg[2], 'o', color = 'lime', markersize = 10)
                print("plotted target")
            ax.scatter(xPosition, yPosition, zPosition, 'bo')
            ax.set_xlabel("X [m]")
            ax.set_ylabel("Y [m]")
            ax.set_zlabel("Altitude [m]")
            plt.plot(0, 0, 0, 'ro')
            ax.quiver(xPositionSounding, yPositionSounding, zPositionSounding, soundingArrayX, soundingArrayY, 0, length = 500, color = 'orange')
        
    plt.show()

    return np.array(waypointSuccesses)




def printResult(waypointSuccesses, opt):
    """
    printResult - find the optimal waypoint coordinate (if any) and print the result onto the commandline

    :param waypointSuccesses: List of Reachable waypoints with corresponding index
    :param opt: option digit ranging from 01 - 30
    """ 

    if waypointSuccesses is not None and np.size(waypointSuccesses) > 0:

        #Initialize empty array
        latitude = []
        longitude = []
        deviation = []
        labels = np.array(["A", "B", "C", "D", "E", "F"])

        #choose success waypoint that has highest remaining altitdue for loiter
        best_idx = np.where(waypointSuccesses[:,1] == max(waypointSuccesses[:,1])) 
        optimal_waypoint_idx = waypointSuccesses[best_idx, 0]

        #read target coordinate and extract data
        with open ("soundAndTarg/targets/predict_" +  opt + "_targets.txt", "r") as f:
            next(f)
            waypointsRaw = f.read().split('\n')
            for i in waypointsRaw:
                array = i.split(",")

                #Handling NaN value in dataset
                if len(array) < 4:
                    continue

                latitude.append(float(array[2]))
                longitude.append(float(array[1]))
                deviation.append(float(array[3]))

        latitude = np.array(latitude)
        longitude = np.array(longitude)
        deviation = np.array(deviation)
        
        #print result
        print("The optimal waypoint is", labels[optimal_waypoint_idx.astype(int)])
        print("Latitude: ", latitude[optimal_waypoint_idx.astype(int)])
        print("Longtitude: ", longitude[optimal_waypoint_idx.astype(int)])
        print("Deviation: ", deviation[optimal_waypoint_idx.astype(int)])

    else:
        print("No optimal waypoint found at the current node's position.")