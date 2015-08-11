import pso_pid
import os
import time

pid = pso_pid.PID()
pid.k = .005
pid.t_i = 10000
pid.printme = True

while(True):
	os.system("clear")
	setpoint = 1500
	position = 1400
	command = pid.update(position, setpoint)
	print "Command:\t",command
	time.sleep(.05)
