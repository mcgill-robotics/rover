import serial_interface as serialInt


if __name__ == "__main__":
    s = serialInt.SerialInterface("/dev/ttyACM0", 9600, timeout=5)
    while True:
        s.send_bytes('0', [34, 54,23,66])
        data = s.read_bytes()
        print(data)

    