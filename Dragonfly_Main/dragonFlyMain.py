import DragonFlyFunctions as dff
import Waypoint_Select_Optimization.WaypointSelectFunctions as waypointSelect
import RPi.GPIO as GPIO #for raspberryPi
import numpy as np
from dronekit import connect, LocationGlobal, VehicleMode
import time
import adafruit_bme680
import board


# --- Initializing Optimal Waypoint (pre - launch) ---
#Retrieve current position from GPS module
lat = 34.45538982594716
lon = -104.7905032972698
alt = 0
optimalWaypoint = np.array([lat, lon, alt]) #Lat (deg), Long (deg), Atl (meters)

# --- Initializing BME680 driver ---
'''
print("Initializing BME680 sensor...")
# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()   # uses board.SCL and board.SDA
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)


# change this to match the location's pressure (hPa) at sea level
bme680.sea_level_pressure = 1013.25
print("Completed.")
'''

# --- Initializing Pixhawk Connection ---
print("Connecting to Vehicle...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)
print("Connected.")



# --- Intitalize loop control variable ---
doJitter = False
senseDeploy = False
Deployed = False
SuccessWaypointFound = False

#________________________________

# print(vehicle.gps_0.fix_type)
# GNSSfix Type:
# 0: no fix
# 1: dead reckoning only
# 2: 2D-fix
# 3: 3D-fix
# 4: GNSS + dead reckoning combined, 
# 5: time only fix

altitude = 0

# During the ascend
while not Deployed:

    #[SIM]
    #Simulating Altitude Gain
    altitude = altitude + 100
    vehicle.location.global_frame.alt = altitude
    # print("Current node's position:", vehicle.location.global_frame)
    time.sleep(0.1)
    #[SIM]

    #[REAL]
    '''
    altitude = float(bme680.altitude)
    '''
    #[REAL]
    
    if altitude < 36000:
        doJitter = True
        senseDeploy = False

    elif altitude >= 36000:
        doJitter = True
        senseDeploy = True

    if doJitter:
        dff.jitterer(vehicle)
        
    if senseDeploy:
        Deployed = dff.deploymentCheck(vehicle)

print("Deployed!")
#________________________________

# During the initial Dive
while Deployed:

    #[SIM]
    #Simulating Altitude Gain
    altitude = altitude + 100
    vehicle.location.global_frame.alt = altitude
    # print("Current node's position:", vehicle.location.global_frame)
    time.sleep(0.1)
    #[SIM]

    #[REAL]
    '''
    altitude = float(bme680.altitude)
    '''
    #[REAL]

    if altitude > 9000 and (vehicle.gps_0.fix_type == 2 or vehicle.gps_0.fix_type == 3):
        
        #Retrieve current position from GPS module
        node_lat = float(vehicle.location.global_frame.lat)
        node_lon = float(vehicle.location.global_frame.lon)
        node_alt = float(vehicle.location.global_frame.alt)
        currentPositionLatLongAlt = np.array([node_lat, node_lon, node_alt]) #Lat (deg), Long (deg), Atl (meters)

        newOptimalWaypoint, Reachable = waypointSelect.run(currentPositionLatLongAlt)

        # if new waypoint is reachable
        if Reachable:
            orig_lat = optimalWaypoint[0]
            orig_lon = optimalWaypoint[1]
            orig_dist = waypointSelect.latlon2distance(node_lat, node_lon, orig_lat, orig_lon)
            
            new_lat = newOptimalWaypoint[0]
            new_lon = newOptimalWaypoint[1]
            new_dist = waypointSelect.latlon2distance(node_lat, node_lon, new_lat, new_lon)

            #if the new waypoint is closer
            if new_dist < orig_dist:
                optimalWaypoint = newOptimalWaypoint


    elif altitude <= 9000:

        #Retrieve current position from GPS module
        node_lat = float(vehicle.location.global_frame.lat)
        node_lon = float(vehicle.location.global_frame.lon)
        node_alt = float(vehicle.location.global_frame.alt)

        dff.deployWings(vehicle)
        time.sleep(2)

        #Arm the vehicle
        vehicle.armed = True

        #Set Mode to Autonomous
        vehicle.mode = VehicleMode("AUTO")

        # Set the multiple LocationGlobal to head towards
        latitudes, longitudes, altitudes = dff.linearInterpolation(5, optimalWaypoint[0], optimalWaypoint[1], node_lat, node_lon, node_alt)
        
        waypoint_locations = []
        for goto_lat, goto_lon, goto_alt in zip(latitudes, longitudes, altitudes):
            waypoint_locations.append(LocationGlobal(goto_lat, goto_lon, goto_alt))

        #for location in waypoint_locations
        vehicle.simple_goto()


        
        
        # dff.streamerRetract()
        
    if altitude < 300:

        print("Chute Deploy")
        dff.chuteDeploy(vehicle)


        

