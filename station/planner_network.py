import socket, os

class PlannerNetworkManager:

	def __init__(self):
		# Delete old sockets from which this was the server
		try:
			os.remove("/tmp/screen_buffer")
		except OSError:
			pass
			
		# Create the socket objects for each connection
		self.lidar_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		
		# Bind each socket to its local UNIX file
		self.lidar_socket.bind("/tmp/screen_buffer")
		
		# Listen on each connection
		self.lidar_socket.listen(5)
				
		# Grab the connection on each of the sockets once the connection is made
		self.lidar_conn, addr = self.lidar_socket.accept()
		
	def shutdown(self):
		self.lidar_conn.close()
		
	def sendLidarString(self, s):
		self.lidar_conn.send(s)
		
	def getUICommand(self):
		try:
			return self.lidar_conn.recv(1024, socket.MSG_DONTWAIT)
		except:
			pass
		
