import serial
import queue
import threading
import sys

#TODO Create Queues priority I/O

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

    WINDOW_SIZE = 32
    next_frame_id = 0

    def __init__(self, port, baudRate):
        self.port = port
        self.baudRate = baudRate
        self.mcu = None
        self.send_queue = queue.Queue()
        self.receive_queue = queue.Queue()
        self.stop = threading.Event()
        self.window = ['']*self.WINDOW_SIZE
        self.SYS_ID = ''
        self.next_frame_id = 0

    def start_link(self):
        """Start connection with the paired device

        Return
        --------
        bool
            Status of the establishment of the link
        """
        # Initialize port connection
        try:
            self.mcu = serial.Serial(self.port, self.baudRate, timeout=1)
        except serial.SerialException:
            return False
        # Establish threads
        self.send_thread = threading.Thread(target=self.send_manager)
        self.send_thread.start()
        self.receive_thread = threading.Thread(target=self.receive_manager)
        self.receive_thread.start()

        #TODO Add frame exchange to let other device know exchange can begin
        return True

    def stop_link(self):
        """Close connection with the paired device
        """
        #TODO Add frames to Stop communication exchange with MCU

        #Stop threads
        self.stop.set()
        self.send_thread.join()
        self.receive_thread.join()

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
        for char in frame:
            while(self.mcu.out_waiting): pass
            try:
                sent = self.mcu.write(char.encode('utf-8'))
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
        str
            valid frame received 
        
        Raises
        --------
        ValueError
            Raised when the CRC8-CCITT of the frame doesn't match the computed one
            Argument is the frame ID of the received frame
        """
        read_byte = ''
        while(read_byte != '~'):
            read_byte = self.mcu.read(1).decode('utf-8')
        frame = f'{read_byte}'      # Should be ~ or it shouldn't get here...
        while(read_byte != '#'):    # Read until end delimiter
            read_byte = self.mcu.read(1).decode('utf-8')
            frame = f'{frame}{read_byte}'
        
        # Check CRC8
        for char in frame[4:-1]:
            crc = compute_crc8ccitt(crc,int.from_bytes(char.encode('utf-8'), byteorder='big'))      #compute for entire payload
        msg_crc = int.from_bytes(frame[3].encode('utf-8'), byteorder='big')
        if(crc != msg_crc):
            raise ValueError            # Raise error if CRCs don't match
        
        frame = f'{frame}{read_byte}'   # Should append # to frame
        return frame

    def send_manager(self):
        """Thread loop for grabbing frames from the queue 
        """
        current_frame = None
        while not self.stop.is_set():
            if(current_frame is not None):      # We are trying to send the head but window is blocking
                if(self.window[int.from_bytes(current_frame[2].encode('utf-8'),byteorder='big')]==''):
                    self.send_frame(current_frame)
                    self.window[int.from_bytes(current_frame[2].encode('utf-8'),byteorder='big')]=current_frame
                    current_frame = None
            elif not self.send_queue.empty():   # Send the next available frame to send
                next_frame = self.send_queue.get()
                if(self.window[int.from_bytes(next_frame[2].encode('utf-8'),byteorder='big')]==''):
                    self.send_frame(next_frame)
                    self.window[int.from_bytes(next_frame[2].encode('utf-8'),byteorder='big')]=next_frame
                else:
                    current_frame = next_frame

    def receive_manager(self):
        """Thread for placing received frames into a queue for processing
        """
        previous_frame_id = 0
        while(not self.stop.is_set()):
            if(self.mcu.in_waiting):
                try:
                    received_frame = self.receive_frame()
                    sys_id_str, frame_id_str, frame_type_str, payload_str = unpackage_frame(received_frame)
                    frame_id = int.from_bytes(frame_id_str.encode('utf-8'),byteorder='big')
                    
                    # Check if frames were lost
                    if((previous_frame_id+1)<frame_id):
                        for j in range(frame_id-previous_frame_id):
                            self.next_frame_id = (self.next_frame_id + 1) % self.WINDOW_SIZE
                            next_frame_id_str = chr(self.next_frame_id)
                            request_frame_id = chr(previous_frame_id+1+j)
                            frame = package_frame(sys_id_str, next_frame_id_str, 'R', request_frame_id)
                            self.send_queue.put(frame)
                    # Preprocess Protocol specific frames
                    if(frame_type_str=='A'):
                        start_window = int.from_bytes(payload_str.encode('utf-8'),byteorder='big')
                        end_window = self.next_frame_id
                        for i in range((start_window-end_window)%self.WINDOW_SIZE):     # Circularly remove all outside window
                            self.window[(end_window+i)%self.WINDOW_SIZE]=''
                    elif(frame_type_str=='R'):
                        self.send_queue.put(self.window[int.from_bytes(payload_str.encode('utf-8'),byteorder='big')])
                    # ACK and place frame in receive queue
                    else:
                        self.next_frame_id = (self.next_frame_id + 1) % self.WINDOW_SIZE
                        next_frame_id_str = chr(self.next_frame_id)
                        ack = package_frame(self.SYS_ID, next_frame_id_str, 'A', frame_id_str)
                        self.send_queue.put(ack)
                        self.receive_queue.put(received_frame)
                        
                    previous_frame_id = frame_id
                except ValueError as err:
                    self.next_frame_id = (self.next_frame_id + 1) % self.WINDOW_SIZE
                    next_frame_id_str = chr(self.next_frame_id)
                    frame = package_frame(sys_id_str, next_frame_id_str, 'R', err.args)
                    self.send_queue.put(frame)

    def send_packet(self, frame_type, packet):
        """Add packet to send to the output frame queue after packaging
        the packet into the frame
        """
        frame = package_frame(self.SYS_ID, self.next_frame_id, frame_type, packet)
        self.next_frame_id = (self.next_frame_id +1)%32
        self.send_queue.put(frame)

    def receive_packet(self):
        """Receive packet from oldest available non-processed frame

        Returns
        --------
        str or None
            valid frame packet 
        """
        if(not self.receive_queue.empty()):
            frame = self.receive_queue.get()
            return unpackage_frame(frame)
        return None

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

def package_frame(sys_id, frame_id, frame_type, payload):
    """Builds the frame following the protocol standard

    Parameters
    --------
    sys_id : str
        system ID field of the frame
    frame_id : int
        frame ID field of the frame
    frame_type : str
        frame type field to decode the payload
    payload : str
        payload field of the frame

    Returns
    --------
    frame : bytes
        frame of proper form following the standard
    """
    frame_id_str = chr(frame_id)
    crc = 0
    crc = compute_crc8ccitt(crc,int.from_bytes(frame_type.encode('utf-8'), byteorder='big'))    #compute for frame type
    for char in payload:
        crc = compute_crc8ccitt(crc,int.from_bytes(char.encode('utf-8'), byteorder='big'))      #compute for entire payload
    crc_str = chr(crc)
    frame = f"~{sys_id}{frame_id_str}{crc_str}{frame_type}{payload}#"
    return frame

def unpackage_frame(frame):
    """Separates fields of the frame to be easily used

    Parameters
    --------
    frame : str
        valid frame to unpackage
    
    Returns
    --------
    sys_id : str
        system ID field of the frame
    frame_id : int
        frame ID field of the frame
    frame_type : str
        frame type field to decode the payload
    payload : str
        payload field of the frame
    """
    sys_id = frame[1]
    frame_id = ord(frame[2])
    frame_type = frame[4]
    payload = frame[5:-1]
    return sys_id, frame_id, frame_type, payload

def compute_crc8ccitt(crc, data):
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
    return int.from_bytes(g_pui8Crc8CCITT[index], byteorder='big')

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
