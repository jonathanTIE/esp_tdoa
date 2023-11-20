import serial

#read and print serial
ser = serial.Serial('COM4', 115200)
while True:
    print (ser.readline())