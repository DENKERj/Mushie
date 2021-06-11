# sudo apt-get update
# sudo apt-get upgrade
# sudo apt-get install python3-dev python3-pip
# sudo python3 -m pip install --upgrade pip setuptools wheel
# sudo pip3 install Adafruit_DHT

# !/usr/bin/env python3
import RPi.GPIO as GPIO
import serial
import time
import Adafruit_DHT

DHT_SENSOR = Adafruit_DHT_AM2302
DHT_PIN = 4
relay_pin = 21
fan_pin = 26
# Set the global variables that will trigger reactions.


def check_humidity(raw):
    if True:
        humidity, temperature = Adafruit_DHT_AM2302.read_retry(DHT_SENSOR, DHT_PIN)

        if humidity is not None and temperature is not None:
            write_to_file("Temp={0:0.1f}*C  Humidity={1:0.1f}%".format(temperature, humidity))
            raw = humidity

        else:
            write_to_file("Failed to retrieve data from humidity sensor")


# This is the co2 sensor.
# First it :actualizes: the sensor
# The first function returns the value of the sensor
# The second function calculates if the fan should turn on
# Then sets the bool
class CO2Sensor():
    request = [0xff, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79]

    def __init__(self, port='/dev/ttyS0'):
        self.serial = serial.Serial(
            port=port,
            baudrate=9600,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

    def get(self):
        self.serial.write(bytearray(self.request))
        response = self.serial.read(9)
        if len(response) == 9:
            current_time = time.strftime('%H:%M:%S', time.localtime())
            return {"time": current_time, "ppa": (response[2] << 8) | response[3], "temp": response[4]}
        return -1


def activate_c02(sensor_d):
    # other Pi versions might need CO2Sensor('/dev/ttyAMA0')
    sensor = CO2Sensor()
    sensor_d = sensor.get()
    return sensor_d


def write_to_file(data):
    log_file = open("log.txt", "a")
    log_file.writelines(data)
    log_file.close()


def object_on(pin):
    GPIO.output(pin, GPIO.HIGH)


def object_off(pin):
    GPIO.output(pin, GPIO.low)


def main():
    # Set Variables
    sensor_data_co2 = 0
    humidity_data = 0
    humidity_desired_min = 75
    humidity_desired_max = 80
    c02_desired_min = 500
    c02_desired_max = 1000

    check_humidity(humidity_data)
    if humidity_desired_min < humidity_data < humidity_desired_max or humidity_data > humidity_desired_max:
        object_off(relay_pin)
    else:
        object_on(relay_pin)

    # Main CO2 Controller
    activate_c02(sensor_data_co2)
    if sensor_data_co2 > c02_desired_max:
        object_on(fan_pin)
    elif c02_desired_max > sensor_data_co2 > c02_desired_min:
        object_off(fan_pin)


if __name__ == '__main__':
    main()
