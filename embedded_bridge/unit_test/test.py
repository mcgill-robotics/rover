import serial_interface

if __name__ == "__main__":
    # Find the Arduino Serial
    found = False
    while not found:
        found, ports = find_arduino_ports()

    s = SerialInterface(ports[0], 9600, timeout=5)
    