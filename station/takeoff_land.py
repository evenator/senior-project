"""DEPRECATED. This was a proof of concept script for taking off and landing.
For an example of sending commands to the drone using the current framework, see
ui_standin.py."""

import socket
import time

ip="192.168.1.1"
port=5556
sock = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
seq=1
command=290718208

print("Sending Takeoff Command");
for x in range(1):
    at_command='AT*REF={0},{1}\r'.format(seq,command)
    result=sock.sendto(at_command, (ip, port) )
    print(at_command+'  ->  {0}'.format(result))
    seq+=1
    time.sleep(.03)
command=290717696
time.sleep(10)
print("Sending Land Command")
for x in range(1):
    at_command='AT*REF={0},{1}\r'.format(seq,command);
    result=sock.sendto(at_command, (ip, port) )
    print(at_command+'  ->  {0}'.format(result))
    seq+=1
    time.sleep(.03)
