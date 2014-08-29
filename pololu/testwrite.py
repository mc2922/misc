import serial
ser = serial.Serial('/dev/ttyS0', 9600)
ser.write("hello\n");
ser.close()
