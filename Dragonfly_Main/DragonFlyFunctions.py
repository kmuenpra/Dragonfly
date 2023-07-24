import adafruit_bme680
import board
#import statement
import RPi.GPIO as GPIO  
from dronekit import connect, mavutil
from time import sleep, time     # this lets us have a time delay (see line 15) 

def deploymentCheck(vehicle):

    GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
    GPIO.setup(25, GPIO.IN)    # set GPIO25 as input (button)  

    #Establish Connection
    # vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)  
    # print("Vehicle is connect")

    #Create "Move Servo" message command
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0, 7, 2000, 1, 1, 0, 0, 0)
    start = time.time()
    timeLapse = time.time() - start
    while timeLapse - start < 10:            # this will carry on until you hit CTRL+C  
        timeLapse = time.time() - start
        print(GPIO.input(25))
        if GPIO.input(25) == GPIO.LOW: # if port 25 == 0 (Falling edge to the ground)
            vehicle.send_mavlink(msg) # send command to servo in order to deploy the node

        sleep(0.005)         # wait 1.1 seconds  



def jitterer():
    # Create sensor object, communicating over the board's default I2C bus
    i2c = board.I2C()   # uses board.SCL and board.SDA
    bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
    # change this to match the location's pressure (hPa) at sea level
    bme680.sea_level_pressure = 1013.25

    jitterTemp = 40 #jitter temperature in Celsius
    print("\nTemperature: %0.1f C" % bme680.temperature)
    if bme680.temperature <= jitterTemp:
        print("jittering!")