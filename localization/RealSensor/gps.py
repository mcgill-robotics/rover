import serial

ser = serial.Serial('/dev/ttyUSB0', 4800, timeout = 5)

while True:
    line = ser.readline()
    splitline = line.split(",")

    if splitline[0] == "$GPGGA":
	latitude = line[2]
	latDirec = line[3]
	longitude = line[4]
	longDirec = line[5]
	print line
	break
