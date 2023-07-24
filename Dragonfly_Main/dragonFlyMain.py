import DragonFlyFunctions as dff
import RPi.GPIO as GPIO
import numpy as np
from dronekit import connect

print("Connecting to Vehicle...")
vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)
print("Connected")

alt = vehicle.location.global_frame.alt

doJitter = False
senseDeploy = False
Deployed = False

dff.deploymentCheck(vehicle)
dff.jitterer()

# while True:

#     if altitude < 36000:
#         doJitter = True
#         senseDeploy = False
#     else if altitude >= 36000:
#         doJitter = True
#         senseDeploy = True
    
#     if doJitter:
#         jitterer()
#     if senseDeploy:
#         DeploymentCheck()

