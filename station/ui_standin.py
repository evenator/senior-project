"""An example of a barebones standin for the UI.
Connects to the PSO, sends a trim command, and sends a liftoff command. After 
waiting five seconds for the drone to finish taking off, sets the mode to manual
override and spins in place for a couple of seconds (sending command updates every
50 ms), then lands the drone.
"""

from pso_network import UISender, PlannerSender
import time
import socket
import math

running=True
pso=UISender()
planner = PlannerSender()
planner.add_waypoint(0,0,500, math.radians(0))
#pso.trim_drone()
pso.set_mode(False)
while True:
	cmd = raw_input('-->')
	pso.trim_drone()
	time.sleep(1)
	pso.reset_pso()
	time.sleep(.05)
	pso.liftoff_drone()
	print "liftoff"
	cmd = raw_input('-->')
	planner.add_waypoint(0,5000,500, math.radians(0))
	cmd = raw_input('-->')
	pso.land_drone()
	print "land"
