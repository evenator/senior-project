import socket
#Setup UDP
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Conf. Host and Port
HOSTNAME = '192.168.1.1'  #drone ip, will be assigned by router
PORT = 5555
RCVVID = 1  #boolean flag for receiving nav data, may need to change into a monitor or semaphore or other process control method

#Connect
socket.connect((HOSTNAME, PORT))
try:
	MSG = "  "  #initiates video data stream
	socket.send(MSG)
	#ack control command
	#MSG = "AT*CTRL=0"
	#socket.send(MSG)
except socket.error,  msg:
    print "Exception, Killing",  msg;


#socket.bind( (HOSTNAME,PORT) )

while True:
	data, addr = socket.recvfrom( 1024 ) # buffer size is 1024 bytes
	print "received message:", data	
		

socket.close()
