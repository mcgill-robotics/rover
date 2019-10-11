import serial

class SerialInterface:
    def __init__(self, port, baudRate):
        self.port = port
        self.baudRate = baudRate
        self.ser = serial.Serial(port, baudRate, timeout=1)
    
    def receive(self):
        data = self.read()
        # Don't print ack if received ACK or nothing
        if data != b'' and data != b'ACK':
            self.write("ACK")
			return data

    def read(self):
        result = ""
        data = self.ser.readline()
        print(data)

        start = data.find(b'[')
        if start == -1:
            return ""
        while data.find(b'[') != -1:
            start = data.find(b'[')

        end = data.find(b']')
        if end == -1:
            return ""

        if start < end:
            result = data[start:end]

        return result

    def send(self):
        pass

    def write(self, message):
        message = "[" + message + "]"
        message = message.encode("utf-8")
        # Wait till the buffer is empty before pushing a new message
        self.ser.flush()
        self.ser.write(message)

    def close(self):
        self.ser.close()

