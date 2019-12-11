import serial
import sys
import time

#Change port/baud rate later
arduino = serial.Serial('COM9', 9600, timeout=.1)
time.sleep(3) #give the connection a second to settle
# arduino.write("Hello from Python!".encode())


def throw():
    for i in range(10):
        arduino.write(1)
    for i in range(25):
        arduino.write("".encode())

    return

throw()
