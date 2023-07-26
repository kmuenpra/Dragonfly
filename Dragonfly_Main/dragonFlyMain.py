import DragonFlyFunctions as dff
import RPi.GPIO as GPIO
import numpy as np
from dronekit import connect
import time

print("Connecting to Vehicle...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)
print("Connected")

alt = vehicle.location.global_frame.alt

doJitter = False
senseDeploy = False
Deployed = False
SuccessWaypointFound = False


print(vehicle.gps_0.fix_type)
# GNSSfix Type:
# 0: no fix
# 1: dead reckoning only
# 2: 2D-fix
# 3: 3D-fix
# 4: GNSS + dead reckoning combined, 
# 5: time only fix

altitude = 0

while not Deployed:

    altitude = altitude + 100
    time.sleep(0.1)

    if altitude < 36000 and not Deployed:
        doJitter = True
        senseDeploy = False

    elif altitude >= 36000 and not Deployed:
        doJitter = True
        senseDeploy = True

    
    if doJitter:
        dff.jitterer()
    if senseDeploy:
        Deployed = dff.deploymentCheck(vehicle)


while Deployed:

    
    if altitude > 9000 and (vehicle.gps_0.fix_type == 2 or vehicle.gps_0.fix_type == 3):
        print("Run Waypoint selection")
        

    elif altitude <= 9000:
        print("Send waypoint to pixhawk")
        print("Change Flight Mode")
        print("Deploy Wings")
        print("Reel in Streamer")

while Gliding:
    if altitude < 300:
        print("Chute Deploy")
        

