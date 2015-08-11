import socket
#Setup UDP
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Conf. Host and Port
HOSTNAME = '192.168.1.1'  #drone ip, will be assigned by router
PORT = 5555
RCVNAV = 1  #boolean flag for receiving nav data, may need to change into a monitor or semaphore or other process control method

#Connect
socket.connect((HOSTNAME, PORT))
try:
	MSG = "AT*CONFIG=\"general:navdata_demo\",\"TRUE\"\\r"  #initiates navadata stream
	socket.send(MSG)
	#ack control command
	MSG = "AT*CTRL=0"
	socket.send(MSG)
except socket.error,  msg:
    print "Exception, Killing",  msg;

try:
	while RCVNAV  #need to figure out what to base the flag on, perhaps the sending of a landing command from the PSO
		#receive data
	
		

socket.close()
