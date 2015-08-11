from pso_network import UISender, PlannerSender
import time
import socket
import math

running=True
pso=UISender()

while True:
	print pso.get_state()
	pso.update(-1.,-.5,0.,.5)
	time.sleep(1)
