#coding: utf-8
"""
A module for the PSO/control loop for the Drone project.

Functions:
main() -- contains the control/PSO loop and is executed when this file is executed

"""

import cv
import numpy
import Queue
import os
import socket
import struct
import copy
import navdata
from threading import Thread, Lock
from safedict import ThreadSafeDict
from drone import Drone
import time
import pso_network
from pso_network import EMERGENCY, SHUTDOWN, FLYING, OVERRIDE, COMMAND, TRIM, RESET
from pso_kalman import PsoKalman
import math
import debuglogger
import pso_pid

def world2drone(x, y, heading):
	drone_x = x * math.cos(heading) - y * math.sin(heading)
	drone_y = x * math.sin(heading) + y * math.cos(heading)
	return (drone_x, drone_y)


def main(): #Main Control Loop (as prototyped on 2/26 in Glennan Lounge)
	"""Main function that runs when the module is called from the command line.
	Starts a ui_listener and a drone object, then runs a loop with updates every
	50 ms. On each loop iteration, check the ui_state, update the PSO, and calculate
	command (for waypoint mode) or pass gamepad command (for manual override mode).
	"""
	# Create listener to receive info from UI
	ui_listener = pso_network.UIListener()
	ui_listener.daemon = True
	ui_listener.start()
	ui_state = ui_listener.get_ui()
	
	# Create listener to recieve waypoints and corrections from planner.
	planner_listener = pso_network.PlannerListener()
	planner_listener.daemon = True
	planner_listener.start()
	waypoint = cv.CreateMat(4, 1, cv.CV_32FC1)
	
	#Instatiate Drone Objects (defined in Drone.py)
	myDrone = Drone("192.168.1.1")
	
	
	#Preset flags
	running = True
	wait_on_emergency = False
	wait_on_liftoff = False
	wait_on_land = False
	
	#Create Kalman filter, state, and command vectors
	kalman = PsoKalman()
	u = cv.CreateMat(4, 1, cv.CV_32FC1)
	z = cv.CreateMat(5, 1, cv.CV_32FC1)
	sys_time = time.time()
	
	#Create PID controllers for each axis
	yaw_pid = pso_pid.PID()
	yaw_pid.k = 1.5
	yaw_pid.t_i = 1.
	yaw_pid.angular = True
	yaw_pid.deadband = .05
	
	z_pid = pso_pid.PID()
	z_pid.k = .00075
	z_pid.i_enable = False
	z_pid.t_i = 10.
	z_pid.deadband = 150
	
	roll_pid = pso_pid.PID()
	roll_pid.k = .00025
	roll_pid.i_enable = False
	roll_pid.deadband = 50
	
	pitch_pid = pso_pid.PID()
	pitch_pid.k = .00025
	pitch_pid.i_enable = False
	pitch_pid.deadband = 50
	
	#Logger puts state in csv for matlab-y goodness
	logger = debuglogger.Logger()
	
	#Fig bucking loop
	while(running):
		time.sleep(.05)
		os.system("clear")
		
		#Get command state from UI
		prev_ui_state = ui_state
		ui_state = ui_listener.get_ui()
				
		if ui_state[EMERGENCY]:
			myDrone.emergency()
		
		if ui_state[SHUTDOWN]:
			#UI has ordered shutdown
			print "Shutting down control loop..."
			ui_listener.stop()
			myDrone.kill()
			running = False
		
		if ui_state[TRIM]:
			myDrone.trim()
			ui_listener.clear_flag(TRIM)
			print "\nTRIM\n"
		
		if ui_state[FLYING]:
			myDrone.takeoff()
			print "Taking Off/Flying"
			if not prev_ui_state[FLYING]:
				wait_on_liftoff = 5
		else:
			myDrone.land()
			print "Landing/Landed"
			if prev_ui_state[FLYING]:
				wait_on_land = 5
		
		if ui_state[RESET]:
			myDrone.reset()
			yaw_pid.flush()
			z_pid.flush()
			roll_pid.flush()
			pitch_pid.flush()
			ui_listener.clear_flag(RESET)
		
		#Get navdata
		nav = myDrone.get_navdata()
		
		#Print out Drone State
		if nav.check_state(navdata.EMERGENCY):
			print "Emergency!"
		elif not nav.check_state(navdata.COM_WATCHDOG):
			print "WATCHDOG"
		elif nav.check_state(navdata.COMMAND):
			print "Watchdog cleared. Not yet ready for commands."
		else:
			print "Ready to Fly\n"
		print "\t\tECACAVNAPCUWAPTHLGCMBNTTTCUACVVF\n{0}\t\t{1:32b}".format(nav.seq,nav.state) #Print navdata state
		
		#Update State (Kalman)
		dt = time.time()-sys_time
		print "dt:\t",dt
		sys_time = time.time()
		z[0, 0], z[1, 0], z[2, 0], z[3, 0], z[4, 0] = nav.vx, nav.vy, nav.z, nav.vz, nav.psi
		#z and u need to be cv matrices!!!!
		sys_state = myDrone.get_state()
		print "\nDrone Kalman State:"
		print "x:\t{0}".format(sys_state[0, 0])
		print "y:\t{0}".format(sys_state[2, 0])
		print "z:\t{0}".format(sys_state[4, 0])
		print "vx:\t{0}".format(sys_state[1, 0])
		print "vy:\t{0}".format(sys_state[3, 0])
		print "vz:\t{0}".format(sys_state[5, 0])
		print "theta:\t{0}".format(sys_state[6, 0])
		print "vtheta:\t{0}".format(sys_state[7, 0])
		
		print "\nNavdata Euler Angles:"
		print "theta:\t",nav.theta
		print "phi:\t",nav.phi
		print "psi:\t",nav.psi
		print "\nNavdata Stuff:"
		print "z:\t",nav.z
		print "vx:\t",nav.vx
		print "vy:\t",nav.vy
		ui_listener.set_state(sys_state, nav)
		logger.log(sys_state)
		
		if wait_on_liftoff>0:
			print "Waiting for liftoff to finish"
			wait_on_liftoff -= dt
			u[0, 0], u[1, 0], u[2, 0], u[3, 0] = 0, 0, 1, 0#Assume drone goes full speed up when taking off
		elif ui_state[FLYING]:
			print "" #Blank line to everything lines up
			#If Drone is in waypoint mode, compute command
			if not ui_state[OVERRIDE]:
				#Get waypoint
				if not planner_listener.waypoints.empty():
					waypoint = planner_listener.waypoints.get()
				print "\nNext Waypoint:"
				print "X:\t", waypoint[0, 0]
				print "Y:\t", waypoint[1, 0]
				print "Z:\t", waypoint[2, 0]
				print "θ:\t", waypoint[3, 0]
				#Compute command
				(roll_des, pitch_des) = world2drone(waypoint[0, 0]-sys_state[0, 0], waypoint[1, 0]-sys_state[2, 0], sys_state[6, 0])
				print "Desired Roll:\t", roll_des
				print "Desired Pitch:\t", pitch_des
				u[0, 0] = pitch_pid.update(0, pitch_des)
				u[1, 0] = roll_pid.update(0, roll_des)
				u[2, 0] = z_pid.update(sys_state[4, 0], waypoint[2, 0])
				u[3, 0] = yaw_pid.update(sys_state[6, 0], waypoint[3, 0])
				myDrone.go(u[0, 0], u[1, 0], u[3, 0], u[2, 0])
			else: #Manual override: Use command from UI state
				print "\nManual override mode\n\n\n"
				myDrone.go(ui_state[COMMAND][0], ui_state[COMMAND][1], ui_state[COMMAND][2], ui_state[COMMAND][3])
				u[0, 0], u[1, 0], u[2, 0], u[3, 0] = ui_state[COMMAND]
		else:
			print "\nLanded"
		
		#Print out commands
		print "\nCommands:\npitch:\t",u[0, 0]
		print "roll:\t", u[1, 0]
		print "gaz:\t", u[2, 0]
		print "yaw:\t", u[3, 0]

if __name__ == "__main__":
	main()
