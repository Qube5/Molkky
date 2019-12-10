import serial
import sys
import time

#Change port/baud rate later
arduino = serial.Serial('COM1', 9600, timeout=.1)
time.sleep(1) #give the connection a second to settle
arduino.write("Hello from Python!")

def throw():
    for i in range(25):
        arduino.write("throw".encode())
    arduino.write("".encode())

    return
