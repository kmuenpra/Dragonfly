import RPi.GPIO as GPIO  
from dronekit import connect, mavutil
from time import sleep     # this lets us have a time delay (see line 15)  
GPIO.setmode(GPIO.BCM)     # set up BCM GPIO numbering  
GPIO.setup(25, GPIO.IN)    # set GPIO25 as input (button)  

vehicle = connect('/dev/serial0', wait_ready=True, baud=921600)  
print("Vehicle is connect")
msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_REPEAT_SERVO, 0, 7, 1500, 1, 1, 0, 0, 0)

try:
    while True:            # this will carry on until you hit CTRL+C  
        if GPIO.input(25): # if port 25 == 1  
            vehicle.send_mavlink(msg)
        sleep(1.1)         # wait 0.1 seconds  
finally:
    GPIO.cleanup()         # clean up after yourself  