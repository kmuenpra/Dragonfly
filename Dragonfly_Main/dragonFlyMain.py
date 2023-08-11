import DragonFlyFunctions as dff
import Waypoint_Select_Optimization.WaypointSelectFunctions as waypointSelect
import RPi.GPIO as GPIO #for raspberryPi
import numpy as np
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time
import adafruit_bme680
import board
import csv


# --- Initializing Optimal Waypoint (pre-launch) ---
# Use best L-1 target
lat = 34.350426
lon = -104.659969
alt = 0
optimalWaypoint = np.array([lat, lon, alt]) #lat [deg], long [deg], alt [meters]


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
# plug PI into telem2 port on pixhawk


# --- Initialize Servo Parameters ---
dff.initializeServos(vehicle)


# --- Intitalize loop control variable ---
deployed = False
SuccessWaypointFound = False
glide = False
dive = False
chute = False


# --- FUNCTIONS --- 

# print(vehicle.gps_0.fix_type)
# GNSSfix Type:
# 0: no fix
# 1: dead reckoning only
# 2: 2D-fix
# 3: 3D-fix
# 4: GNSS + dead reckoning combined, 
# 5: time only fix


# --- Create Data File ---
# open the file in the write mode
f = open('data.csv', 'w')

# create the csv writer
writer = csv.writer(f)
writer.writerow('BME altitude', 'node lat', 'node long', 'node alt', 'velo', 'attitude', 'battery', 'deployed', 'glide', 'dive', 'chute')

# write a row to the csv file
def write():
    node_lat = float(vehicle.location.global_frame.lat)
    node_lon = float(vehicle.location.global_frame.lon)
    node_alt = float(vehicle.location.global_frame.alt)
    writer.writerow(float(bme680.altitude), node_lat, node_lon, node_alt, vehicle.velocity, vehicle.attitude, vehicle.battery, deployed, glide, dive, chute)


# --- Ascent ---
origAltitude = 1220     # altitude of Ft Sumner [m]

# --- Simulate Altitude --- (COMMENT OUT)
def altitudeSim(prev):
    if not deployed:
        # add 1000 meters everytime altitude is called
        altitude = prev + 1000
    else:
        altitude = prev - 1000

    print("Altitude: " + altitude)
    return altitude
altitude = altitudeSim(origAltitude)

# disable all control surfaces for ascent
dff.ascentSet(vehicle)

# repeat until note senses deployment
while not deployed:
    #[SIM]
    altitude = altitudeSim(altitude)
    #[SIM]

    #[REAL]
    '''
    altitude = float(bme680.altitude)
    '''
    #[REAL]
    
    if altitude > 10000:
        dff.jitterer(vehicle)
        deployed = dff.deploymentCheck(vehicle)

    write()
    time.sleep(5) # change for actual flight

print("Deployed!")
#________________________________


# --- Descent ---
while deployed:
    #[SIM]
    altitude = altitudeSim(altitude)
    #[SIM]

    if not dive:
        dff.diveMode(vehicle)
        dive = True

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

        # Interpolate two coordiantes to get multiple waypoints to navigate towards
        latitudes, longitudes, altitudes = dff.linearInterpolation(5, optimalWaypoint[0], optimalWaypoint[1], node_lat, node_lon, node_alt)
        
        #Create LocationGlobal object
        waypoints_locations = []
        for goto_lat, goto_lon, goto_alt in zip(latitudes, longitudes, altitudes):
            waypoints_locations.append(LocationGlobal(goto_lat, goto_lon, goto_alt))

        #Upload new mission
        cmds = vehicle.commands

        print(" Clear any existing commands")
        cmds.clear() 

        print(" Define/add new commands.")
        for waypoint in waypoints_locations:                                                                                                    #lat          #lon          #alt
            cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, waypoint[0], waypoint[1], waypoint[2]))

        print(" Upload new commands to vehicle")
        cmds.upload()

        if not glide:
            dff.glideMode(vehicle)
            dff.deployWings(vehicle)
            glide = True

            #Arm the vehicle
            vehicle.armed = True

            #Set Mode to Autonomous
            vehicle.mode = VehicleMode("AUTO")

        
    if altitude < 1000 + origAltitude:
        dff.chuteDeploy(vehicle)
        print("Chute Deploy")

    write()
    time.sleep(5)


# close the file
f.close()