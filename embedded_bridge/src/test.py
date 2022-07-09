import serial_interface as serialInt
import random
import time


if __name__ == "__main__":
    s = serialInt.SerialInterface("/dev/ttyACM0", 9600, timeout=5)
    while True:
        time.sleep(2)
        s.send_bytes('0', [2.2423784237844, 3, 5, 7])
        data = s.read_bytes()
        print(f"Received: {data}")
    