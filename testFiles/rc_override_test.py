from dronekit import connect, time, mavutil, VehicleMode

vehicle = connect('/dev/tty.usbserial-D30EZ7F3', wait_ready=True, baud = 57600)


print("Vehicle connect")

'''
# Get all vehicle attributes (state)
print("\nGet all vehicle attribute values:")
print(" Autopilot Firmware version: %s" % vehicle.version)
print("   Major version number: %s" % vehicle.version.major)
print("   Minor version number: %s" % vehicle.version.minor)
print("   Patch version number: %s" % vehicle.version.patch)
print("   Release type: %s" % vehicle.version.release_type())
print("   Release version: %s" % vehicle.version.release_version())
print("   Stable release?: %s" % vehicle.version.is_stable())
print(" Autopilot capabilities")
print("   Supports MISSION_FLOAT message type: %s" % vehicle.capabilities.mission_float)
print("   Supports PARAM_FLOAT message type: %s" % vehicle.capabilities.param_float)
print("   Supports MISSION_INT message type: %s" % vehicle.capabilities.mission_int)
print("   Supports COMMAND_INT message type: %s" % vehicle.capabilities.command_int)
print("   Supports PARAM_UNION message type: %s" % vehicle.capabilities.param_union)
print("   Supports ftp for file transfers: %s" % vehicle.capabilities.ftp)
print("   Supports commanding attitude offboard: %s" % vehicle.capabilities.set_attitude_target)
print("   Supports commanding position and velocity targets in local NED frame: %s" % vehicle.capabilities.set_attitude_target_local_ned)
print("   Supports set position + velocity targets in global scaled integers: %s" % vehicle.capabilities.set_altitude_target_global_int)
print("   Supports terrain protocol / data handling: %s" % vehicle.capabilities.terrain)
print("   Supports direct actuator control: %s" % vehicle.capabilities.set_actuator_target)
print("   Supports the flight termination command: %s" % vehicle.capabilities.flight_termination)
print("   Supports mission_float message type: %s" % vehicle.capabilities.mission_float)
print("   Supports onboard compass calibration: %s" % vehicle.capabilities.compass_calibration)
print(" Global Location: %s" % vehicle.location.global_frame)
print(" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print(" Local Location: %s" % vehicle.location.local_frame)
'''

# pixhawk channel guide
#   1 elevon left
#   2 elevon right
#   3 rudder 1
#   4 rudder 2
#   5 small actuators 
#   6 big actuator
#   7 parachute
#   8 esc (powers the rail)

CHANNELS = {
    'LeftElevon': '1',
    'RightElevon': '2',
    'Rudder1': '3',
    'Rudder2': '4',
    'Deployment': '5',
    'WingFolding': '6',
    'Parachute': '7'
}




vehicle.parameters['SERVO1_FUNCTION'] = 78
print(vehicle.parameters['SERVO1_FUNCTION'])




# deploy wings
msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 6, 1000,0, 0, 0, 0, 0)
vehicle.send_mavlink(msg)



vehicle.flush()
vehicle.close()

'''
# Move small linear actuator   [ch 5]
msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1700,0, 0, 0, 0, 0)
print("Deploying Node")
vehicle.send_mavlink(msg)

# Move big linear actuator  [ch 6]





#vehicle.channels.overrides['4'] = 1300
#vehicle.channels.overrides['1'] = 1400 


msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1, 1300,0, 0, 0, 0, 0)
vehicle.send_mavlink(msg)

msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 2, 1300,0, 0, 0, 0, 0)
vehicle.send_mavlink(msg)



#vehicle.send_mavlink(msg)


print("read: %s" % vehicle.channels['1'])
print(vehicle.channels)



# Send the channel overrides to the vehicle
vehicle.flush()

# Wait for a few seconds
time.sleep(5)

# Reset channel overrides
vehicle.channels.overrides = {}
vehicle.flush()

# Close the connection
vehicle.close()
'''