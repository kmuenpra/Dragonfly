from site import check_enableusersite

#import statement
import adafruit_bme680
import board
import RPi.GPIO as GPIO  
from dronekit import connect, mavutil
from time import sleep, time     # this lets us have a time delay (see line 15) 
import numpy as np

def deploymentCheck(vehicle):

    GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
    GPIO.setup(25, GPIO.IN)    # set GPIO25 as input (button)  

    #Create malvink message command to retract the small actuators (for deployment system)
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000,0, 0, 0, 0, 0)
    
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



def jitterer(vehicle):
    # Create sensor object, communicating over the board's default I2C bus
    i2c = board.I2C()   # uses board.SCL and board.SDA
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    
    # change this to match the location's pressure (hPa) at sea level
    bme680.sea_level_pressure = 1013.25

    jitterTemp = 40 #jitter temperature in Celsius

    print("\nTemperature: %0.1f C" % bme680.temperature)

    if bme680.temperature <= jitterTemp:
        channels = [1,2,4,5,6]
        pwm = [1600, 1600, 1600 , 1900, 1900]
        for i in range(len(channels)):                                                                               #ch,     PWM,   Repeats, Delays
            msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0, channels[i],  pwm[i],   1,      0,    0, 0, 0)
            #print("jittering on channel " + channels[i])
            vehicle.send_mavlink(msg)
            sleep(1)

def streamerRetract():
    GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering
    GPIO.setup(13, GPIO.OUT) #set pin 13 as output

    retractingTime = 20
    print("Retracting Streamer")
    start = time.time()
    timeLapse = time.time() - start
    while timeLapse < retractingTime:
        
        timeLapse = time.time() - start
        GPIO.output(13, True)
    
    GPIO.output(13, False)
    print("Streamer Retracted :)")

def deployWings(vehicle,pwm):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 6, pwm, 0, 0, 0, 0, 0)
    print("Deploying Wings")
    vehicle.send_mavlink(msg)

def chuteDeploy(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7, 2000, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    print("Deploying Chute")

def chuteReset(vehicle):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 7, 1000, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    print("Reseting Chute")

def runMotorStreamer():
    pin = 6			# PWM pin connected to LED
    GPIO.setwarnings(False)			#disable warnings
    GPIO.setmode(GPIO.BCM)		#set pin numbering system
    GPIO.setup(pin,GPIO.OUT)
    while True:
        GPIO.output(pin, True)
        print("True")
        sleep(5)
        GPIO.output(pin, False)
        print("not True")
        sleep(5)

def nodeDeploymentTest(vehicle,pwm):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, pwm,0, 0, 0, 0, 0)
    print("Deploying Node")
    vehicle.send_mavlink(msg)

def runMotorStreamer2(vehicle, pwm, waitTime):
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 3, pwm,0, 0, 0, 0, 0)
    msg1 = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 3, 0,0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)
    print("Signal on")
    sleep(waitTime)
    vehicle.send_mavlink(msg1)
    print("Signal off")
    sleep(waitTime)

def linearInterpolation(num_of_points, target_lat, target_lon, current_lat, current_lon, current_alt):

    latitudes = np.linspace(current_lat, target_lat, num_of_points)
    longitudes = (target_lon - current_lon)/(target_lat - current_lat)*(latitudes - current_lat) + current_lon
    altitudes = np.linspace(current_alt, 0 , num_of_points)

    return latitudes, longitudes, altitudes
