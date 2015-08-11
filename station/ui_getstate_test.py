import numpy
import cv
import pso_network
import navdata

receiver = pso_network.UIListener()
receiver.daemon = True
receiver.start()

state_in = cv.CreateMat(8, 1, cv.CV_32FC1)
state_in[0, 0] = 0
state_in[1, 0] = 1
state_in[2, 0] = 2
state_in[3, 0] = 3
state_in[4, 0] = 4
state_in[5, 0] = 5
state_in[6, 0] = 6
state_in[7, 0] = 7
nav_in = navdata.Navdata()
nav_in.battery = 90
nav_in.state = navdata.FLYING | navdata.EMERGENCY

print "Input:"
#Print out Drone State
print "\t\tECACAVNAPCUWAPTHLGCMBNTTTCUACVVF\n{0}\t\t{1:32b}".format(nav_in.seq,nav_in.state) #Print navdata state
print "Battery:\t",nav_in.battery
print "x:\t{0}".format(state_in[0, 0])
print "y:\t{0}".format(state_in[2, 0])
print "z:\t{0}".format(state_in[4, 0])
print "vx:\t{0}".format(state_in[1, 0])
print "vy:\t{0}".format(state_in[3, 0])
print "vz:\t{0}".format(state_in[5, 0])
print "theta:\t{0}".format(state_in[6, 0])
print "vtheta:\t{0}".format(state_in[7, 0])

receiver.set_state(state_in, nav_in)
while True:
	receiver.get_ui()
