import serial

#TODO Create Queues for I/O and priority I/O
#TODO Create Receive frame construction function

class SerialInterface:
    """Serial Interface to establish connection and communicate
    with devices using the McGill Robotics Rover Serial Frame standard

    Attributes
    --------
    port : str
        port to connect to external hardware with
    baudrate : int
        data rate used to communicate with paired device
    mcu : serial.Serial
        serial object interfacing with the hardware level
    """

    def __init__(self, port, baudRate):
        self.port = port
        self.baudRate = baudRate
        self.mcu = None

    def start_link(self):
        """Start connection with the paired device

        Return
        --------
        bool
            Status of the establishment of the link
        """
        try:
            self.mcu = serial.Serial(self.port, self.baudRate, timeout=1)
        except serial.SerialException:
            return False
        #TODO Add frame exchange to let other device know exchange can begin
        return True

    def stop_link(self):
        """Close connection with the paired device
        """
        #TODO Add frames to Stop communication exchange with MCU
        self.mcu.close()

    def send_frame(self, frame):
        """Sends out frame byte by byte to the paired device

        Parameters
        --------
        frame : str
            frame with data of all fields of the frame ready and already parsed for sending
        
        Returns
        --------
        bool
            whether frame has been properly sent out
        """
        counter = 0
        frame = F"~{frame}#"
        for char in frame:
            while(self.mcu.out_waiting): pass
            try:
                sent = self.mcu.write(char.encode('ascii'))
                if(sent<1):
                    raise serial.SerialException
            except (serial.SerialException, serial.SerialTimeoutException) as e:
                    #TODO Establish way of resetting connection
                    pass
            counter = counter + 1
        return (counter == len(frame))

    def receive_frame(self):
        """Receive frame from paired device byte by byte if available

        Returns
        --------
        str or None
            frame received 
        """
        pass

    def __del__(self):
        """Deconstructor of object
        Closes open serial port
        """
        if(self.mcu is not None and self.mcu.is_open):
            self.mcu.close()
            self.mcu = None

def find_ports():
    """Finds all ports available for connection

    Returns
    --------
    list(str)
        all ports available for conneciton
    """
    port_list = []
    if(sys.platform == "win32" or sys.platform == "win64"):
        for i in range(0,128):
            port = "COM"+str(i)
            if(port_is_valid(port)):
                port_list.append(port)
        
    elif(sys.platform == "linux" or sys.platform == "linux2"):
        for i in range(0,128):
            port = "/dev/ttyUSB"+str(i)
            if(port_is_valid(port)):
                port_list.append(port)
            else:
                break
        for i in range(0,128):
            port = "/dev/ttyACM"+str(i)
            if(port_is_valid(port)):
                port_list.append(port)
            else:
                break
    return port_list

def port_is_valid(port):
    """Verifies if a port is valid 
    Performs trial connection,fails if port doesn't exist

    Returns
    --------
    bool
        whether port is valid or not
    """
    try:
        s = serial.Serial(port)
        s.close()
        return True
    except serial.SerialException:
        return False

def compute_crc8ccitt(self, crc, data):
    """Compute the CRC8-CCITT of the data for Error Flow

    Parameters
    --------
    crc : int
        Current value of the CRC8-CCITT
    data : int
        next value in the payload
    
    Returns
    --------
    int 
        new computed crc value
    """
    index = crc ^ data
    return int(g_pui8Crc8CCITT[index])

g_pui8Crc8CCITT = [
    b'\x00', b'\x07', b'\x0E', b'\x09', b'\x1C', b'\x1B', b'\x12', b'\x15',
    b'\x38', b'\x3F', b'\x36', b'\x31', b'\x24', b'\x23', b'\x2A', b'\x2D',
    b'\x70', b'\x77', b'\x7E', b'\x79', b'\x6C', b'\x6B', b'\x62', b'\x65',
    b'\x48', b'\x4F', b'\x46', b'\x41', b'\x54', b'\x53', b'\x5A', b'\x5D',
    b'\xE0', b'\xE7', b'\xEE', b'\xE9', b'\xFC', b'\xFB', b'\xF2', b'\xF5',
    b'\xD8', b'\xDF', b'\xD6', b'\xD1', b'\xC4', b'\xC3', b'\xCA', b'\xCD',
    b'\x90', b'\x97', b'\x9E', b'\x99', b'\x8C', b'\x8B', b'\x82', b'\x85',
    b'\xA8', b'\xAF', b'\xA6', b'\xA1', b'\xB4', b'\xB3', b'\xBA', b'\xBD',
    b'\xC7', b'\xC0', b'\xC9', b'\xCE', b'\xDB', b'\xDC', b'\xD5', b'\xD2',
    b'\xFF', b'\xF8', b'\xF1', b'\xF6', b'\xE3', b'\xE4', b'\xED', b'\xEA',
    b'\xB7', b'\xB0', b'\xB9', b'\xBE', b'\xAB', b'\xAC', b'\xA5', b'\xA2',
    b'\x8F', b'\x88', b'\x81', b'\x86', b'\x93', b'\x94', b'\x9D', b'\x9A',
    b'\x27', b'\x20', b'\x29', b'\x2E', b'\x3B', b'\x3C', b'\x35', b'\x32',
    b'\x1F', b'\x18', b'\x11', b'\x16', b'\x03', b'\x04', b'\x0D', b'\x0A',
    b'\x57', b'\x50', b'\x59', b'\x5E', b'\x4B', b'\x4C', b'\x45', b'\x42',
    b'\x6F', b'\x68', b'\x61', b'\x66', b'\x73', b'\x74', b'\x7D', b'\x7A',
    b'\x89', b'\x8E', b'\x87', b'\x80', b'\x95', b'\x92', b'\x9B', b'\x9C',
    b'\xB1', b'\xB6', b'\xBF', b'\xB8', b'\xAD', b'\xAA', b'\xA3', b'\xA4',
    b'\xF9', b'\xFE', b'\xF7', b'\xF0', b'\xE5', b'\xE2', b'\xEB', b'\xEC',
    b'\xC1', b'\xC6', b'\xCF', b'\xC8', b'\xDD', b'\xDA', b'\xD3', b'\xD4',
    b'\x69', b'\x6E', b'\x67', b'\x60', b'\x75', b'\x72', b'\x7B', b'\x7C',
    b'\x51', b'\x56', b'\x5F', b'\x58', b'\x4D', b'\x4A', b'\x43', b'\x44',
    b'\x19', b'\x1E', b'\x17', b'\x10', b'\x05', b'\x02', b'\x0B', b'\x0C',
    b'\x21', b'\x26', b'\x2F', b'\x28', b'\x3D', b'\x3A', b'\x33', b'\x34',
    b'\x4E', b'\x49', b'\x40', b'\x47', b'\x52', b'\x55', b'\x5C', b'\x5B',
    b'\x76', b'\x71', b'\x78', b'\x7F', b'\x6A', b'\x6D', b'\x64', b'\x63',
    b'\x3E', b'\x39', b'\x30', b'\x37', b'\x22', b'\x25', b'\x2C', b'\x2B',
    b'\x06', b'\x01', b'\x08', b'\x0F', b'\x1A', b'\x1D', b'\x14', b'\x13',
    b'\xAE', b'\xA9', b'\xA0', b'\xA7', b'\xB2', b'\xB5', b'\xBC', b'\xBB',
    b'\x96', b'\x91', b'\x98', b'\x9F', b'\x8A', b'\x8D', b'\x84', b'\x83',
    b'\xDE', b'\xD9', b'\xD0', b'\xD7', b'\xC2', b'\xC5', b'\xCC', b'\xCB',
    b'\xE6', b'\xE1', b'\xE8', b'\xEF', b'\xFA', b'\xFD', b'\xF4', b'\xF3'
]
