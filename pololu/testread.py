import serial
ser = serial.Serial('/dev/ttymxc3', 9600)
line = ser.readline();
print line
ser.close()
