import socket
import time

port = 5556 #port to send on
ip = '192.168.1.1'
sock=socket.socket( socket.AF_INET, socket.SOCK_DGRAM )

seq=1
try:
	while True:
		sock.sendto("AT*REF={0},290718208\r".format(seq), (ip, port))
		seq += 1
		time.sleep(.02)
except:
	pass
sock.sendto("AT*REF={0},256\r".format(seq), (ip, port))
sock.close()
