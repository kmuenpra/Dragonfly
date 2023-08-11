
# --- Import statements ---

from site import check_enableusersite
#import adafruit_bme680
#import board
#import RPi.GPIO as GPIO  
from dronekit import connect, mavutil, VehicleMode
from time import sleep, time     # this lets us have a time delay (see line 15) 
#import numpy as np


# --- Define Channels ---
CHANNELS = {
    'ElevonLeft': '1',
    'ElevonRight': '2',
    'RudderTop': '3',
    'RudderBottom': '4',
    'Deployment': '5',
    'WingFolding': '6',
    'Parachute': '7'
}


# --- Functions ---

def initializeServos(vehicle):
    # Left Elevon
    vehicle.parameters['SERVO1_MIN'] = 1100
    vehicle.parameters['SERVO1_MAX'] = 2000
    vehicle.parameters['SERVO1_TRIM'] = 1380

    # Right Elevon
    vehicle.parameters['SERVO2_MIN'] = 1100
    vehicle.parameters['SERVO2_MAX'] = 2000
    vehicle.parameters['SERVO2_TRIM'] = 1380

    # Top Rudder
    vehicle.parameters['SERVO3_MIN'] = 1000
    vehicle.parameters['SERVO3_MAX'] = 2000
    vehicle.parameters['SERVO3_TRIM'] = 1310

    # Bottom Rudder
    vehicle.parameters['SERVO4_MIN'] = 1000
    vehicle.parameters['SERVO4_MAX'] = 2000
    vehicle.parameters['SERVO4_TRIM'] = 1310

    # Deployment
    vehicle.parameters['SERVO5_MIN'] = 1000
    vehicle.parameters['SERVO5_MAX'] = 2000
    vehicle.parameters['SERVO5_REVERSED'] = 0
    vehicle.parameters['SERVO5_FUNCTION'] = 0

    # Wing Folding
    vehicle.parameters['SERVO6_MIN'] = 1000
    vehicle.parameters['SERVO6_MAX'] = 2000
    vehicle.parameters['SERVO6_REVERSED'] = 0
    vehicle.parameters['SERVO6_FUNCTION'] = 0

    # Parachute
    vehicle.parameters['SERVO7_MIN'] = 1000
    vehicle.parameters['SERVO7_MAX'] = 2000
    vehicle.parameters['SERVO7_REVERSED'] = 0
    vehicle.parameters['SERVO7_FUNCTION'] = 0

def deploymentCheck(vehicle):

    GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
    GPIO.setup(25, GPIO.IN)    # set GPIO25 as input (button)  

    #Create malvink message command to retract the small actuators (for deployment system)
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Deployment']), 1000,0, 0, 0, 0, 0)
    
    start = time.time()
    timeLapse = time.time() - start
    signalLength = 0.040 #40 ms signal required for drop
    signalcounter = 0 #measuring length of deployment signal
    
    while timeLapse - start < 10: #run the deploymentCheck command for 10 seconds
        
        timeLapse = time.time() - start
        #print(GPIO.input(25))

        if GPIO.input(25) == GPIO.LOW: # if port 25 == 0 (Falling edge to the ground)
            signalcounter = signalcounter + 0.001
        else:
            signalcounter = 0
        
        if signalcounter >= signalLength: #check if the system receive a consecutive signal to drop the node
            vehicle.send_mavlink(msg) # send command to servo in order to deploy the node
            return True

        sleep(0.001)         # wait 0.001 seconds

    return False


def ascentSet(vehicle):
    # requires elevons and rudder to be disabled
    setElevonLeftFunction(vehicle,0)
    setElevonRightFunction(vehicle,0)
    setRudderTopFunction(vehicle,0)
    setRudderBottomFunction(vehicle,0)

    # set each to their trim condition
    setElevonLeftPWM(vehicle, 'trim')     # left elevon
    setElevonRightPWM(vehicle, 'trim')    # right elevon
    setRudderTopPWM(vehicle, 'trim')      # top rudder
    setRudderBottomPWM(vehicle, 'trim')   # bottom rudder


def printElevonLeftFunction(vehicle):
    print(vehicle.parameters['SERVO1_FUNCTION'])

def setElevonLeftFunction(vehicle,mode):
    vehicle.parameters['SERVO1_FUNCTION'] = mode

def setElevonLeftPWM(vehicle,pwm):
    # MAKE SURE SERVO1_FUNCTION IS 0 (DISABLED)
    # run setElevonLeftFunction(vehicle,mode) = 0 if testing in isolation

    if pwm == 'trim':
        pwm = 1380

    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['ElevonLeft']), pwm, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def printElevonRightFunction(vehicle):
    print(vehicle.parameters['SERVO2_FUNCTION'])

def setElevonRightFunction(vehicle,mode):
    vehicle.parameters['SERVO2_FUNCTION'] = mode

def setElevonRightPWM(vehicle,pwm):
    # MAKE SURE SERVO2_FUNCTION IS 0 (DISABLED)
    # run setElevonRightFunction(vehicle,mode) = 0 if testing in isolation

    if pwm == 'trim':
        pwm = 1380
    
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['ElevonRight']), pwm, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def printRudderTopFunction(vehicle):
    print(vehicle.parameters['SERVO3_FUNCTION'])

def setRudderTopFunction(vehicle,mode):
    vehicle.parameters['SERVO3_FUNCTION'] = mode

def setRudderTopPWM(vehicle,pwm):
    # MAKE SURE SERVO3_FUNCTION IS 0 (DISABLED)
    # run setRudderTopFunction(vehicle,mode) = 0 if testing in isolation

    if pwm == 'trim':
        pwm = 1310

    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['RudderTop']), pwm, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

def printRudderBottomFunction(vehicle):
    print(vehicle.parameters['SERVO4_FUNCTION'])

def setRudderBottomFunction(vehicle,mode):
    vehicle.parameters['SERVO4_FUNCTION'] = mode

def setRudderBottomPWM(vehicle,pwm):
    # MAKE SURE SERVO4_FUNCTION IS 0 (DISABLED)
    # run setRudderBottomFunction(vehicle,mode) = 0 if testing in isolation

    if pwm == 'trim':
        pwm = 1310

    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['RudderBottom']), pwm, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)


def jitterer(vehicle):
    # Create sensor object, communicating over the board's default I2C bus
    i2c = board.I2C()   # uses board.SCL and board.SDA
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    
    # change this to match the location's pressure (hPa) at sea level
    bme680.sea_level_pressure = 1013.25

    jitterTemp = 40 #jitter temperature in Celsius

    print("\nTemperature: %0.1f C" % bme680.temperature)

    if bme680.temperature <= jitterTemp:
        jitter(vehicle)

def jitter(vehicle):
    i = 0
    trim = [1380, 1380, 1310, 1310, 2000, 1000]
    pwm1 = [trim[0]+50, trim[1]+50, trim[2]+50, trim[3]+50, trim[4]-50, trim[5]+50]
    pwm2 = [trim[0]-50, trim[1]-50, trim[2]-50, trim[3]-50, trim[4], trim[5]]
    while i < 6:
        msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, i+1, pwm1[i],0,0,0,0,0)
        vehicle.send_mavlink(msg)
        sleep(1)
        msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, i+1, pwm2[i],0,0,0,0,0)
        vehicle.send_mavlink(msg)
        sleep(1)
        i = i + 1

def testJitter(vehicle):
    vehicle.parameters['SERVO1_FUNCTION'] = 0
    vehicle.parameters['SERVO2_FUNCTION'] = 0
    vehicle.parameters['SERVO3_FUNCTION'] = 0
    vehicle.parameters['SERVO4_FUNCTION'] = 0

    i = 0
    while i < 10:
        jitter(vehicle)
        i = i + 1

def deployWings(vehicle,pwm):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['WingFolding']), pwm, 0, 0, 0, 0, 0)
    print("Deploying Wings")
    vehicle.send_mavlink(msg)

def chuteDeploy(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Parachute']), 2000, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    print("Deploying Chute")

def chuteReset(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Parchute']), 1000, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    print("Reseting Chute")

def nodeDeploymentTest(vehicle,pwm):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, int(CHANNELS['Deployment']), pwm,0, 0, 0, 0, 0)
    print("Deploying Node")
    vehicle.send_mavlink(msg)

def linearInterpolation(num_of_points, target_lat, target_lon, current_lat, current_lon, current_alt):
    latitudes = np.linspace(current_lat, target_lat, num_of_points)
    longitudes = (target_lon - current_lon)/(target_lat - current_lat)*(latitudes - current_lat) + current_lon
    altitudes = np.linspace(current_alt, 0 , num_of_points)

    return latitudes, longitudes, altitudes

def changeModes(vehicle, left, right, top, bottom):
    vehicle.parameters['SERVO1_FUNCTION'] = left
    vehicle.parameters['SERVO2_FUNCTION'] = right
    vehicle.parameters['SERVO3_FUNCTION'] = top
    vehicle.parameters['SERVO4_FUNCTION'] = bottom

def aileronsMode(vehicle):
    vehicle.parameters['SERVO1_FUNCTION'] = 4
    vehicle.parameters['SERVO2_FUNCTION'] = 4
    vehicle.parameters['SERVO3_FUNCTION'] = 0
    vehicle.parameters['SERVO4_FUNCTION'] = 0

    setRudderTopPWM(vehicle,'trim')
    setRudderBottomPWM(vehicle,'trim')

def diveMode(vehicle):
    # left elevon (aileron function)
    vehicle.parameters['SERVO1_REVERSED'] = 1
    vehicle.parameters['SERVO1_FUNCTION'] = 4
    
    # right elevon (aileron function)
    vehicle.parameters['SERVO2_REVERSED'] = 1
    vehicle.parameters['SERVO2_FUNCTION'] = 4

    # top rudder (aileron function)
    vehicle.parameters['SERVO3_REVERSED'] = 1
    vehicle.parameters['SERVO3_FUNCTION'] = 4

    # bottom rudder (aileron function)
    vehicle.parameters['SERVO4_REVERSED'] = 1
    vehicle.parameters['SERVO4_FUNCTION'] = 4


def glideMode(vehicle):
    # right elevon
    vehicle.parameters['SERVO1_REVERSED'] = 1
    vehicle.parameters['SERVO1_FUNCTION'] = 78

    # left elevon
    vehicle.parameters['SERVO2_REVERSED'] = 0
    vehicle.parameters['SERVO2_FUNCTION'] = 77

    # top rudder
    vehicle.parameters['SERVO3_REVERSED'] = 0
    vehicle.parameters['SERVO3_FUNCTION'] = 21

    # bottom rudder
    vehicle.parameters['SERVO4_REVERSED'] = 1
    vehicle.parameters['SERVO4_FUNCTION'] = 21
