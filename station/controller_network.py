import socket, os

class ControllerNetworkManager:

	def __init__(self):
		# Delete old sockets from which this was the server
		try:
			os.remove("/tmp/controller")
		except OSError:
			print "OSERROR"
			pass
			
		# Create the socket objects for each connection
		self.controller_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		
		# Bind each socket to its local UNIX file
		self.controller_socket.bind("/tmp/controller")
		
		# Listen on each connection
		self.controller_socket.listen(5)
				
		# Grab the connection on each of the sockets once the connection is made
		self.controller_conn, addr = self.controller_socket.accept()
		
	def shutdown(self):
		self.controller_conn.close()
		
	def sendControllerString(self, s):
		self.controller_conn.send(s)
		
	def getUICommand(self):
		try:
			return self.controller_conn.recv(1024, socket.MSG_DONTWAIT)
		except:
			pass
		
