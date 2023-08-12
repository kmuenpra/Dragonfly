import DragonFlyFunctions as dff
import Waypoint_Select_Optimization.WaypointSelectFunctions as waypointSelect
#import RPi.GPIO as GPIO #for raspberryPi
import numpy as np
from dronekit import connect, LocationGlobal, VehicleMode, Command, mavutil
import time
#import csv


# --- Initializing Optimal Waypoint (pre-launch) ---
# Use closest target from example (already given)
lat = 34.350426
lon = -104.659969
alt = 0
optimalWaypoint = np.array([lat, lon, alt]) #lat [deg], long [deg], alt [meters]


# --- Initializing Pixhawk Connection ---
print("Connecting to Vehicle...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)
print("Connected.")
# plug PI into telem2 port on pixhawk


# --- Initialize Servo Parameters ---
dff.initializeServos(vehicle)

# disable all control surfaces for ascent
dff.ascentSet(vehicle)


# --- Intitalize loop control variables ---
mounted = False
deployed = False
SuccessWaypointFound = False
glide = False
dive = False
chute = False
falling = False


# print(vehicle.gps_0.fix_type)
# GNSSfix Type:
# 0: no fix
# 1: dead reckoning only
# 2: 2D-fix
# 3: 3D-fix
# 4: GNSS + dead reckoning combined, 
# 5: time only fix


'''
# --- Create Data File --- (doesn't currently work with PI autorun)
# open the file in the write mode
f = open('data.csv', 'w')

# create the csv writer
writer = csv.writer(f)
writer.writerow('node lat, node long, node alt, velo, attitude, battery, deployed, glide, dive, chute')

# write a row to the csv file
def write():
    node_lat = float(vehicle.location.global_frame.lat)
    node_lon = float(vehicle.location.global_frame.lon)
    node_alt = float(vehicle.location.global_frame.alt)
    writer.writerow(node_lat,', ', node_lon,', ', node_alt,', ', vehicle.velocity,', ', vehicle.attitude,', ', vehicle.battery,', ', deployed,', ', glide,', ', dive,', ', chute)
'''


# --- Simulate Altitude --- (REMOVE BEFORE FLIGHT)
origAltitude = 1220     # altitude of Ft Sumner [m]
def altitudeSim(prev):
    if not deployed:
        # add 1000 meters everytime altitude is called
        altitude = prev + 1000
    else:
        # subtract 1000 meters everytime altitude called after deployment
        altitude = prev - 1000

    print("Altitude: ", altitude)
    return altitude
altitude = origAltitude

'''
# --- Mounting ---
# NEED TO HAVE MOUNTING PINS IN
GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(25, GPIO.IN)    # set GPIO25 as input (button)  
dff.nodeDeploymentTest(vehicle,1500)

while not mounted:
    if GPIO.input(25) == GPIO.HIGH:
        dff.nodeDeploymentTest(vehicle,2000)
        mounted = True
'''

# --- Ascent ---
origAltitude = 1400     # average altitude of New Mexico [m]

while not deployed:
    prevAltitude = altitude

    #[SIM]
    altitude = altitudeSim(altitude)
    #[SIM]

    #[REAL]
    '''
    # get altitude from GPS if it is not disconnected
    if vehicle.gps_0.fix_type != 0:
        altitude = vehicle.location.global_frame.alt

        # if node hasn't deployed and altitude dropping, deploy parachute
        if falling == True & altitude < prevAltitude & altitude < (1000 + origAltitude):
            deployed = True
            dff.chuteDeploy()
            deployed = True
        elif altitude < prevAltitude & altitude < (1000 + origAltitude):
            falling = True
        else:
            falling = False
    '''
    #[REAL]
    
    # alt when temperature drops below 0 in sounding files
    if altitude > 6000:
        dff.jitter(vehicle)
        deployed = dff.deploymentCheck(vehicle)

    # write()

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
    # get altitude from GPS if it is not disconnected
    if vehicle.gps_0.fix_type != 0:
        altitude = vehicle.location.global_frame.alt
    '''
    #[REAL]

    if altitude > 9000 and (vehicle.gps_0.fix_type == 2 or vehicle.gps_0.fix_type == 3):

        #Retrieve current position from GPS module
        nodegps = vehicle.location.global_frame
        node_lat = float(nodegps.lat)
        node_lon = float(nodegps.lon)
        node_alt = float(nodegps.alt)
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

    
    # deploy parachute at 500 meters (1600 ft) above original altitude (avg of NM)
    if altitude < (500 + origAltitude):
        dff.chuteDeploy(vehicle)
        print("Chute Deployed")

    #write()
    time.sleep(5)


# close the file
#f.close()