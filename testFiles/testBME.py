from dronekit import connect
import time
import bme680

# Connect to the vehicle
vehicle = connect('/dev/ttyS0', baud=57600, wait_ready=True)

# Initialize the BME680 sensor
sensor = bme680.BME680(bme680.I2C_ADDR_PRIMARY)

# Set up the BME680 sensor
sensor.set_humidity_oversample(bme680.OS_2X)
sensor.set_pressure_oversample(bme680.OS_4X)
sensor.set_temperature_oversample(bme680.OS_8X)
sensor.set_filter(bme680.FILTER_SIZE_3)

try:
    while True:
        if sensor.get_sensor_data():
            temperature = sensor.data.temperature
            humidity = sensor.data.humidity
            print(f"Temperature: {temperature} °C, Humidity: {humidity} %")
        time.sleep(1)

except KeyboardInterrupt:
    pass

# Close the connection
vehicle.close()
