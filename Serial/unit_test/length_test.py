import sys
sys.path.insert(0, '..')
import serialInterface
import time

port = serialInterface.find_ports()
s = serialInterface.SerialInterface(port[0], 9600)
if(s.start_link()):
    i = 0
    while True:
        s.put_packet('0',str(int(i%100)))
        time.sleep(0.1)
        i = (i+1)%100
