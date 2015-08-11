from PyQt4 import *
from PyQt4 import Qt,uic,QtGui,QtNetwork
from PyQt4 import QtCore
import sys, traceback
import cv, numpy
import thread, time
import socket,os
from pso_network import UISender
from controller import Controller
from log import Logger
		
class UI(QtGui.QMainWindow):
	def closeEvent(self, event):
		print "closing"
		time.sleep(5)
		print "Closing"
		QtGui.QMainWindow.closeEvent(self,event)

	def planner_read(self):
		if (self.lidar_socket.bytesAvailable() > 810000):
			data = self.lidar_socket.readData(810000)
			getMapData(self, data)
			
	def video_read(self):
		if (self.video_socket.bytesAvailable() > 230400):
			data = self.video_socket.readData(230400)
			print len(data)
			getVideoData(self, data)
			
	def controller_read(self):
		try:
			cdata = self.controller_socket.readData(1024)
			split = cdata.split(";")
			#print split
			for x in split:
				#print x, "  x"
				val = x.split("=")
				#print val
				if (len(val) == 1):
					#print "caught short val"
					pass
				elif (val[1] == "-"):
					#print "caught minus sign"
					pass
				elif (val[1] == ""):
					#print "caught empty string"
					pass
				elif (x.startswith("LY")):
					#print "###LY", x
					self.__LY = float(val[1]) # error ValueError: empty string for float() on this line
					#print self.__LY
					if (self.__LY < -0.1):
						self.__LS_UP = float(val[1])
						#self.__manualValue[4] = float(val[1])
						#print self.__LS_UP, "LSUP"
					elif (self.__LY > 0.1):
						self.__LS_DOWN = float(val[1])
						#self.__manualValue[5] = float(val[1])
						#print self.__LS_DOWN, "LSDOWN"
					else:  
						self.__LS_DOWN = float(0.000)
						self.__LS_UP = float(0.000)
						#self.__manualValue[4] = float(0.000)
						#self.__manualValue[5] = float(0.000)
						#print self.__LS_DOWN
						#print self.__LS_UP
				elif (x.startswith("LX")):
					#print "###LX", x
					self.__LX = float(val[1])
					#print self.__LX
					if (self.__LX < -0.1):
						self.__LS_LEFT = float(val[1])
						#self.__manualValue[6] = float(val[1])
						#print self.__LS_LEFT, "LSLEFT"
					elif (self.__LX > 0.1):
						self.__LS_RIGHT = float(val[1])
						#self.__manualValue[7] = float(val[1])
						#print self.__LS_RIGHT, "LSRIGHT"
					else:
						self.__LS_RIGHT = float(0.000)
						self.__LS_LEFT = float(0.000)
						#self.__manualValue[6] = float(0.000)
						#self.__manualValue[7] = float(0.000)
						#print self.__LS_LEFT
						#print self.__LS_RIGHT
				elif (x.startswith("LT")):
					self.__LT = float(val[1])
				elif (x.startswith("RY")):
					self.__RY = float(val[1])
					#print self.__RY
					if (self.__RY < -0.1):
						self.__RS_UP = float(val[1])
						#self.__manualValue[8] = float(val[1])
						#print self.__RS_UP, "RSUP"
					elif (self.__RY > 0.1):
						self.__RS_DOWN = float(val[1])
						#self.__manualValue[9] = float(val[1])
						#print self.__RS_DOWN, "RSDOWN"
					else:
						self.__RS_DOWN = float(0.000)
						self.__RS_UP = float(0.000)
						#self.__manualValue[8] = float(0.000)
						#self.__manualValue[9] = float(0.000)
						#print self.__RS_DOWN
						#print self.__RS_UP
				elif (x.startswith("RX")):
					self.__RX = float(val[1])
					#print self.__LX
					if (self.__RX < -0.1):
						self.__RS_LEFT = float(val[1])
						#self.__manualValue[10] = float(val[1])
						#print self.__RS_LEFT, "RSLEFT"
					elif (self.__RX > 0.1):
						self.__RS_RIGHT = float(val[1])
						#self.__manualValue[11] = float(val[1])
						#print self.__RS_RIGHT, "RSRIGHT"
					else:
						self.__RS_RIGHT = float(0.000)
						#self.__manualValue[11] = float(0.000)
						self.__RS_LEFT = float(0.000)
						#self.__manualValue[10] = float(0.000)
						#print self.__RS_LEFT
						#print self.__RS_RIGHT
				elif (x.startswith("RT")):
					self.__RT = float(val[1])
				elif (x.startswith("AA")):
					self.__A = int(val[1])
					#self.__manualValue[14] = int(val[1])
					#print self.__A, "A"
				elif (x.startswith("BB")):
					self.__B = int(val[1])
					#self.__manualValue[15] = int(val[1])
					#print self.__B, "B"
				elif (x.startswith("XX")):
					self.__X = int(val[1])
					#self.__manualValue[12] = int(val[1])
					#print self.__X, "X"
				elif (x.startswith("YY")):
					self.__Y = int(val[1])
					#self.__manualValue[13] = int(val[1])
					#print self.__Y, "Y"
				elif (x.startswith("LB")):
					self.__LB = int(val[1])
					#self.__manualValue[16] = int(val[1])
					#print self.__LB, "LB"
				elif (x.startswith("RB")):
					self.__RB = int(val[1])
					#self.__manualValue[17] = int(val[1])
					#print self.__RB, "RB"
				elif (x.startswith("Back")):
					self.__BACK = int(val[1])
					#self.__manualValue[22] = int(val[1])
					#print self.__BACK, "BACK"
				elif (x.startswith("Start")):
					self.__START = int(val[1])
					#self.__manualValue[23] = int(val[1])
					#print self.__START, "START"
				elif (x.startswith("LS")):
					self.__LS = int(val[1])
					#self.__manualValue[20] = int(val[1])
					#print self.__LS, "LS"
				elif (x.startswith("RS")):
					self.__RS = int(val[1])
					#self.__manualValue[21] = int(val[1]) 
					#print self.__RS, "RS"
				elif (x.startswith("D_pad")):  #guessing on the values for direction, may be incorrect
					self.__D_UP = 0 #reset all dpad values to 0
					self.__D_DOWN = 0
					self.__D_LEFT = 0
					self.__D_RIGHT = 0
					#self.__manualValue[0] = 0
					#self.__manualValue[1] = 0
					#self.__manualValue[2] = 0
					#self.__manualValue[3] = 0
					#print "#####", x  # D_pad=(1,1)
					# D_pad ; (1,1)
					dpad1 = val[1].split("(") #  ; 1,1)
					dpad2 = dpad1[1].split(",") #  1  ; 1)
					dpadtemp = int(dpad2[0])
					if (dpadtemp == 1):
						self.__D_RIGHT = dpadtemp
						#self.__manualValue[3] = dpadtemp
					if (dpadtemp == -1):
						self.__D_LEFT = 1
						#self.__manualValue[2] = 1
					dpad3 = dpad2[1].split(")") # 1 ; 
					dpadtemp1 = int(dpad3[0])
					if (dpadtemp1 == 1):
						self.__D_UP = dpadtemp1
						#self.__manualValue[0] = dpadtemp1
					if (dpadtemp1 == -1):
						self.__D_DOWN = 1
						#self.__manualValue[1] = 1
					#print self.__D_UP, "D_UP"
					#print self.__D_DOWN, "D_DOWN"
					#print self.__D_LEFT, "D_LEFT"
					#print self.__D_RIGHT, "D_RIGHT"
		except Exception as ex:
				print ex
				traceback.print_exc(file=sys.stdout)

		#print cdata
		
		"""
		print self.__RY
		print self.__RX
		print self.__RT
		print self.__LY
		print self.__LX
		print self.__LT
		print self.__A
		print self.__B
		print self.__X
		print self.__Y
		print self.__LB
		print self.__RB
		print self.__BACK
		print self.__START
		print self.__LS
		print self.__RS
		print self.__D_UP
		print self.__D_DOWN
		print self.__D_LEFT
		print self.__D_RIGHT
		print self.__LS_UP
		print self.__LS_DOWN
		print self.__LS_LEFT
		print self.__LS_RIGHT
		print self.__RS_UP
		print self.__RS_DOWN
		print self.__RS_LEFT
		print self.__RS_RIGHT
		"""
		
		#PSO SEND MOVE - hardcoded for now
		

		try:
			if (self.__mode == 2):
				
				if (self.__A == 1):
					print "liftoff"
					self.pso_socket.liftoff_drone()
					self.logger.log("LIFTOFF COMMAND SENT")
				elif (self.__B == 1):
					print "land"
					self.pso_socket.land_drone()
					self.logger.log("LAND COMMAND SENT")
				elif (self.__START == 1):
					print "ESTOP"
					self.pso_socket.emergency_stop()
					self.logger.log("ESTOP COMMAND SENT")
					
				#Dead Band Implementation	
				if (self.__LY < 0.15 and self.__LY > -0.15):
					self.__LY = 0.0
				elif (self.__LY > 0.15):
					self.__LY = self.__LY - 0.15
				elif (self.__LY < -0.15):
					self.__LY = self.__LY + 0.15
				if (self.__LX < 0.15 and self.__LX > -0.15):
					self.__LX = 0.0
				elif (self.__LX > 0.15):
					self.__LX = self.__LX - 0.15
				elif (self.__LX < -0.15):
					self.__LX = self.__LX + 0.15
				if (self.__RY < 0.15 and self.__RY > -0.15):
					self.__RY = 0.0
				elif (self.__RY > 0.15):
					self.__RY = self.__RY - 0.15
				elif (self.__RY < -0.15):
					self.__RY = self.__RY + 0.15
				if (self.__RX < 0.15 and self.__RX > -0.15):
					self.__RX = 0.0
				elif (self.__RX > 0.15):
					self.__RX = self.__RX - 0.15
				elif (self.__RX < -0.15):
					self.__RX = self.__RX + 0.15
				#if (self.__LY != 0.0):
				#	self.__LY = -self.__LY
				#if (self.__RY != 0.0):
				#	self.__RY = -self.__RY
				
				# Trigger Cases
				#  RT = -1  LT = -1  # do nothing assume -1 ~ -.95
				#  RT > -1  LT > -1  # do nothing
				#  RT = -1  LT > -1  # go up  VERT > 0
				#  		-1 < LT < 1  # add 1 to all sides
				#					0 < LT < 2  #divide by 2
				#  RT > -1  LT = -1  # go down VERT < 0
				#		-1 < RT < 1  # add 1 to all sides
				#					0 < RT < 2	#divide by 2
				
				if (self.__RT < -.85 and self.__LT < -.85):
					VERT = float(0.000)
				elif (self.__RT > -.85 and self.__LT > -.85):
					VERT = float(0.000)
				elif (self.__RT < -.85 and self.__LT > -.85):
					tVert1 = (self.__LT + 1)/2
					VERT = -tVert1
				elif (self.__RT > -.85 and self.__LT < -.85):
					tVert2 = (self.__RT + 1)/2
					VERT = tVert2
				#print "VERT:  ", VERT
				
				# send to pso network (pitch, roll, yaw, vertical)
				self.pso_socket.update(self.__LY, self.__LX, self.__RX, VERT)
				print "update", self.__LY, self.__LX, self.__RX, VERT
				self.logger.log("MOVE::"+" Pitch:"+str(self.__LY)+" Roll:"+str(self.__LX)+" Yaw:"+str(self.__RX)+" Gaz:"+str(VERT))
				
				"""
				up = 0
				down = 0
				left = 0
				right = 0
				forward = 0
				backward = 0
				rotateright = 0
				rotateleft = 0
				takeoff = 0
				land = 0
				estop = 0
				cyclecamera = 0
				"""
				
				#for x in self.__manualValue: # get all items in the list that have a value, next need to determine how I'm going to set the value and send 
				#	if (x != 0):
				#		print x
				
				#print self.__manualName
				#print self.__manualValue
			
		except Exception as ex:
		   print ex
		   traceback.print_exc(file=sys.stdout)

 
		
	def planner_connect(self):
		self.logger.log("Planner Connection Established...")
		print "Planner Connection Established..."
		   	
	def planner_error(self):
		self.logger.log("Planner unable to connect")
		print "Planner unable to connect"
		
	def video_connect(self):
		self.logger.log("Video Connection Established...")
		print "Video Connection Established..."
		   	
	def video_error(self):
		self.logger.log("Video unable to connect")
		print "Video unable to connect"
		print self.video_socket.errorString()	
		
	def controller_connect(self):
		self.logger.log("Controller Connection Established...")
		print "Controller Connection Established..."
		
	def controller_error(self):
		self.logger.log("Controller unable to connect")
		print "Controller unable to connect"
		
	def get_navdata(self):
		nav = self.pso_socket.get_state()
		print nav	
			
	def set_manual(self):
		print "manual set"
		self.logger.log("Manual control updated")
		#for j in range(len(self.__manualName)):
			#self.__manualValue[j] = str(self.ui.findChild(Qt.QComboBox, self.__manualName[j]).currentText())
		#for j in range(len(self.__manualName)):
			#print self.__manualName[j]
			#print self.__manualValue[j]
			
	def controller_reconnect(self): #does not work
		print self.controller, "self.controller"
		if (self.controller):
			print "Controller already connected"
		else:
			self.controller = Controller()
			self.controller.daemon = True
			self.controller.start()
		
			self.controller_socket = QtNetwork.QLocalSocket()
			self.controller_socket.connected.connect(self.controller_connect)
			self.controller_socket.readyRead.connect(self.controller_read)
			self.controller_socket.error.connect(self.controller_error)
			self.controller_socket.connectToServer("/tmp/controller")	
		 
		
	def mapMouse(self, event):
		s = "click " + str(event.x()) + " " + str(event.y())
		self.lidar_socket.writeData(s)
		
		
	def modeChanged(self, buttonId):
		checked = buttonId.objectName()
		if (checked == "radAutonomous"):
			#print "checked: ", checked
			self.__mode = 0
			self.lidar_socket.writeData("mode 0")
			self.pso_socket.set_mode(False)
		elif (checked == "radWaypoint"):
			#print "checked: ", checked
			self.__mode = 1
			self.lidar_socket.writeData("mode 1")   
			self.pso_socket.set_mode(False) 		
		elif (checked == "radManual"):
			#print "checked: ", checked
			self.__mode = 2
			self.lidar_socket.writeData("mode 2")
			self.pso_socket.set_mode(True)
		if (self.__mode == 0):
			self.logger.log("Flight Mode -> Autonomous")
			print "Flight Mode:Autonomous"
		elif (self.__mode == 1):
			self.logger.log("Flight Mode -> Waypoint")
			print "Flight Mode:Waypoint"
		elif (self.__mode == 2):
			self.logger.log("Flight Mode -> Manual")
			print "Flight Mode:Manual"

	#This function gets the information from the drone/PSO and displays it in the ui	
	def display_nav_data(self):
		navData =  self.pso_socket.get_state()
		#Data that is displayed as text labels
		self.ui.findChild(QtGui.QLabel, "Altitude").setText(str(round(navData.get("z","--"),2)))
		self.ui.findChild(QtGui.QLabel, "vZ").setText(str(round(navData.get("vz","--"),2)))
		self.ui.findChild(QtGui.QLabel, "vY").setText(str(round(navData.get("vy","--"),2)))
		self.ui.findChild(QtGui.QLabel, "vX").setText(str(round(navData.get("vx","--"),2)))
		self.ui.findChild(QtGui.QLabel, "Yaw").setText(str(round(navData.get("yaw","--"),2)))
		#self.ui.findChild(QtGui.QLabel, "Pitch").setText(str(round(navData.get("pitch","--"),2)))
		#self.ui.findChild(QtGui.QLabel, "Roll").setText(str(round(navData.get("roll","--"),2)))
		self.ui.findChild(QtGui.QProgressBar, "batteryLevel").setValue(int(navData.get("bat",0)))
		#Data that is displayed graphically
		self.yawScene.addEllipse(50,50,95,95)		
		self.pitchScene.addEllipse(50,50,95,95)
		self.rollScene.addEllipse(50,50,95,95)
#		self.yawScene.addLine(10,50,90,50)
		self.ui.findChild(QtGui.QGraphicsView, "graphicsView_5").setScene(self.yawScene)
		self.ui.findChild(QtGui.QGraphicsView, "graphicsView_4").setScene(self.pitchScene)
		self.ui.findChild(QtGui.QGraphicsView, "graphicsView_3").setScene(self.rollScene)
		#log the flight data
		self.logger.log("Nav Data:: Altitude:"+str(navData.get("z","--"))+" vZ:"+str(navData.get("vz","--"))+
			" vY:"+str(navData.get("vy","--"))+" vX:"+str(navData.get("vx","--"))+
			" Bat:"+str(navData.get("bat",0)))
		#print "Display data", navData

	def display_graph(self):
		self.graphScene.addLine(1,105,640,105)
		self.ui.findChild(QtGui.QGraphicsView, "graphicsView").setScene(self.graphScene)
	
	#This function sends the e-stop command and sets the TOL button to take-off	 
	def eStop(self):
		self.pso_socket.emergency_stop()
		self.logger.log("E-Stop Command sent")
		self.ui.findChild(QtGui.QPushButton, "pushButton").setText("Take Off")
		print "E-Stop Sent"

	#This function will get the data from the drone and determine if the drone is flyng.
	#If the drone is flying this will send the command to land the drone and if the 
	#drone is landed  it will send the command to take off.  This will also change the 
	#text of the TOL button to the appropriate command for the drone
	def get_TOL(self):
		print "get_TOL data"
		flight_state = self.pso_socket.get_state()
		flying = flight_state["flying"]
		if(flying == False):
			self.pso_socket.liftoff_drone()
			self.logger.log("Take Off Command Sent")
			#put in a check that the drone has actually taken off
			self.ui.findChild(QtGui.QPushButton, "pushButton").setText("Land")

		if(flying == True):
			self.pso_socket.land_drone()
			self.logger.log("Land Command Sent")
			#put in a check that the drone has actually landed
			self.ui.findChild(QtGui.QPushButton, "pushButton").setText("Take Off")

	def trim_drone(self):
		flight_state = self.pso_socket.get_state()
		flying = flight_state["flying"]
		if(not flying):
			self.logger.log("Drone Trimmed")
			self.pso_socket.trim_drone()

	def export_map(self):
		print "test map export"
		self.lidar_socket.writeData("export")

	#This function resets the information in the PSO and the UI
	def reset(self):
		flight_state = self.pso_socket.get_state()
		#print flight_state
		flying = flight_state["flying"]
		if(not flying):
			self.pso_socket.land_drone()
			self.ui.findChild(QtGui.QPushButton, "pushButton").setText("Take Off")
			self.logger.log("PSO Reset")
			self.pso_socket.reset_pso()

	def update_tilt_box(self):
		self.ui.findChild(QtGui.QDoubleSpinBox, "doubleSpinBox_2").setValue(round(self.ui.findChild(QtGui.QSlider,"maxTiltSlider").value / 10,1))

	def update_tilt_slider(self):
		self.ui.findChild(QtGui.QSlider, "maxTiltSlider").setValue(int(self.ui.findChild(QtGui.QDoubleSpinBox, "doubleSpinBox_2").value * 10))

	def update_timer(self):
		self.timer.start(250)

	def __init__(self):
		self.ui = uic.loadUi("mainwindow.ui")
		#self.control_img = self.ui.findChild(QtGui.QLabel, "control_image")
		#self.control_img.setPixmap(QtGui.QPixmap(":/DroneControl.png"))
		self.ui.show()

		self.logger = Logger()

		self.timer = QtCore.QTimer()
		QtCore.QObject.connect(self.timer,QtCore.SIGNAL("timeout()"),self.update_timer)
		QtCore.QObject.connect(self.timer,QtCore.SIGNAL("timeout()"),self.display_nav_data)
		QtCore.QObject.connect(self.timer,QtCore.SIGNAL("timeout()"),self.display_graph)
		#QtCore.QObject.connect(self.timer,QtCore.SIGNAL("timeout()"),self.controller_reconnect)
		self.update_timer()	

		QtCore.QObject.connect(self.ui.findChild(QtGui.QSlider,"maxTiltSlider"), QtCore.SIGNAL("valueChanged(int)"),
			self.update_tilt_box)
		QtCore.QObject.connect(self.ui.findChild(QtGui.QDoubleSpinBox, "doubleSpinBox_2"), QtCore.SIGNAL("valueChanged(d)"),
			self.update_tilt_slider)

		self.yawScene = QtGui.QGraphicsScene()
		self.pitchScene = QtGui.QGraphicsScene()
		self.rollScene = QtGui.QGraphicsScene()
		self.graphScene = QtGui.QGraphicsScene()
#		self.ui.findChild(QtGui.QGraphicsView, "graphicsView_2").setBackgroundBrush(QtGui.QBrush(":/manual_controls_img.jpg"))
#		self.ui.findChild(QtGui.QGraphicsView, "graphicsView_2").drawBackground()

		self.controller = Controller()
		self.controller.daemon = True
		self.controller.start()
		
		self.lidar_socket = QtNetwork.QLocalSocket()
		self.lidar_socket.connected.connect(self.planner_connect)
		self.lidar_socket.readyRead.connect(self.planner_read)
		self.lidar_socket.error.connect(self.planner_error)
		self.lidar_socket.connectToServer("/tmp/screen_buffer")
		
		self.video_socket = QtNetwork.QLocalSocket()
		self.video_socket.connected.connect(self.video_connect)
		self.video_socket.readyRead.connect(self.video_read)
		self.video_socket.error.connect(self.video_error)
		self.video_socket.connectToServer("/tmp/video")
		
		self.controller_socket = QtNetwork.QLocalSocket()
		self.controller_socket.connected.connect(self.controller_connect)
		self.controller_socket.readyRead.connect(self.controller_read)
		self.controller_socket.error.connect(self.controller_error)
		self.controller_socket.connectToServer("/tmp/controller")	  
		
		self.ui.findChild(QtGui.QGraphicsView, "Map").mousePressEvent = self.mapMouse
		
		
				
		self.pso_socket = UISender()
		
		#self.__manualName = ["d_up", "d_down", "d_left", "d_right", "ls_up", "ls_down", "ls_left", "ls_right", "rs_up", "rs_down", "rs_left", "rs_right", "b_x", "b_y", "b_a", "b_b", "lb", "rb", "lt", "rt", "ls_press", "rs_press", "b_back", "b_start"]
		#self.__manualFunction = []	
		#self.__manualValue = []
		
		
		self.__RY = 0.000
		self.__RX = 0.000
		self.__RT = -1.000
		self.__LY = 0.000
		self.__LX = 0.000
		self.__LT = -1.000
		self.__A = 0
		self.__B = 0
		self.__X = 0
		self.__Y = 0
		self.__LB = 0
		self.__RB = 0
		self.__BACK = 0
		self.__START = 0
		self.__LS = 0
		self.__RS = 0
		self.__D_UP = 0
		self.__D_DOWN = 0
		self.__D_LEFT = 0
		self.__D_RIGHT = 0
		self.__LS_UP = 0.000
		self.__LS_DOWN = 0.000
		self.__LS_LEFT = 0.000
		self.__LS_RIGHT = 0.000
		self.__RS_UP = 0.000
		self.__RS_DOWN = 0.000
		self.__RS_LEFT = 0.000
		self.__RS_RIGHT = 0.000
			
		#for j in range(len(self.__manualName)):
			#print self.__manualName[j]
			#print self.__manualValue[j]
		
			
		#print self.__manualName
		#print self.__manualFunction
		#print self.__manualValue
		
			
		
		self.__mode = 0
		self.ui.findChild(QtGui.QButtonGroup, "modeButtonGroup").buttonPressed.connect(self.modeChanged)
		self.ui.findChild(QtGui.QPushButton, "pushButton").clicked.connect(self.get_TOL)
		self.ui.findChild(QtGui.QPushButton, "pushButton_3").clicked.connect(self.eStop)
		self.ui.findChild(QtGui.QPushButton, "trimPushButton").clicked.connect(self.trim_drone)
		self.ui.findChild(QtGui.QPushButton, "exportMap").clicked.connect(self.export_map)
		self.ui.findChild(QtGui.QPushButton, "resetButton").clicked.connect(self.reset)
		
		"""
		d_up = self.ui.findChild(Qt.QComboBox, "d_up").currentText()
		print d_up
		d_down = self.ui.findChild(Qt.QComboBox, "d_down").currentText()
		print d_down
		d_right = self.ui.findChild(Qt.QComboBox, "d_right").currentText()
		print d_right
		d_left = self.ui.findChild(Qt.QComboBox, "d_left").currentText()
		print d_left
		ls_up = self.ui.findChild(Qt.QComboBox, "ls_up").currentText()
		print ls_up
		ls_down = self.ui.findChild(Qt.QComboBox, "ls_down").currentText()
		print ls_down
		ls_right = self.ui.findChild(Qt.QComboBox, "ls_right").currentText()
		print ls_right
		ls_left = self.ui.findChild(Qt.QComboBox, "ls_left").currentText()
		print ls_left
		"""	   

		
		
def getMapData(self, a):
	
	img = Qt.QImage(450, 450, Qt.QImage.Format_RGB32) #creates qImage
	value = Qt.qRgb(122, 163, 39)  #color of pixel to be drawn
	
	#arr = numpy.fromstring(a, numpy.uint8)
   	img = Qt.QImage(a, 450, 450, Qt.QImage.Format_RGB32)

   	pixmap = Qt.QPixmap.fromImage(img)
   	scene = Qt.QGraphicsScene()
   	scene.addPixmap(pixmap)
   	self.ui.Map.setScene(scene)
   	
   	
def getVideoData(self, a):
	
	#img = Qt.QImage(160, 120, Qt.QImage.Format_RGB888) #creates qImage
	#value = Qt.qRgb(122, 163, 39)  #color of pixel to be drawn
	
	#arr = numpy.fromstring(a, numpy.uint8)
   	img = Qt.QImage(a, 320, 240, Qt.QImage.Format_RGB888)

   	pixmap = Qt.QPixmap.fromImage(img)
   	scene = Qt.QGraphicsScene()
   	scene.addPixmap(pixmap)
   	self.ui.DownVideo.setScene(scene)
   	

 
		
	
	  
def convertIPLtoQImg(ipl):
	height = ipl.height
	width = ipl.width
	#if (ipl.depth == cv.IPL_DEPTH_8U and ipl.nChannels == 3):
	qImageBuffer = ipl.imageData
	img = Qt.QImage(qImageBuffer, width, height, QImage.Format_RGB888)
	return img.rgbSwapped()
	
	
app = QtGui.QApplication(sys.argv)
window = UI()

a = ""
"""
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect("/tmp/screen_buffer")

while 1:
	try:
		data = s.recv(810000)
	   	if data:
	   		a = a + data
	   		if (len(a) == 810000):
				getMapData(window, a)
				a = ""
			if (len(a) > 810000):
				a = ""
	except Exception as ex:
		print ex
	except KeyboardInterrupt:
		break
"""
app.exec_()
		
#conn.close()




