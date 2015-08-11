"""
This module defines a class PID that is a PID controller."""

import time
import numpy
import math

class PID:
	error = numpy.array([0.,0.,0.])
	command = numpy.array([0.,0.,0.])
	setpoint = 0
	angular = False
	printme = False
	
	i_enable = True
	d_enable = False
	
	k = 0.
	t_i = 100.
	t_d = 0.
	dt = .05
	t = 0.
	deadband = 0.
	
	def __init(self, angular = False):
		self.angular = angular
		t = time.time()
	
	def update(self, position, setpoint = None):
		if setpoint is not None:
			self.setpoint = setpoint
		self.dt = time.time()-self.t
		self.t = time.time()
		if self.printme:
			print "k:\t", self.k
			print "Ti:\t", self.t_i
			print "Td:\t", self.t_d
			print "dt:\t", self.dt
			print "Setpoint:\t",self.setpoint
			print "Position:\t",position
		self.command = numpy.roll(self.command, 1)
		self.error = numpy.roll(self.error, 1)
		self.error[0] = self.setpoint - position
		if self.angular:
			while self.error[0] > math.pi+.1: #hysteresis
				self.error[0] -= math.pi
			while self.error[0] < -math.pi-.1: #hysteresis
				self.error[0] += math.pi
		if self.printme:
			print "Error:\t",self.error
		
		if math.fabs(self.error[0]) > self.deadband:
			self.command[0] = self.k *self.error[0]
			if self.i_enable:
				self.command[0] += self.command[1] + self.k * self.dt / self.t_i * self.error[0] - self.k * self.error[1]
				if self.d_enable:
					self.command[0] += self.k * (self.t_d / self.dt * self.error[0] -2 * self.t_d / self.dt * self.error[1] + self.t_d / self.dt * self.error[2])
			elif self.d_enable:
				self.command[0] += self.k * (self.t_d / self.dt * self.error[0] - self.t_d / self.dt * self.error[1])
				
		else:
			self.command[0] = 0
		
		if self.printme:
			print "Integral Component:\t", (self.command[1] + self.k * self.dt / self.t_i * self.error[0] - self.k * self.error[1])
			print "Proportional Component:\t", (self.k * self.error[0])
			print "Raw Command:\t",self.command[0]

		#Saturation
		if self.command[0] > 1:
			self.command[0] = 1
		if self.command[0] < -1:
			self.command[0] = -1
		return self.command[0]
		
	def flush(self):
		self.error = numpy.array([0.,0.,0.])
		self.command = numpy.array([0.,0.,0.])
		self.setpoint = 0
		self.t = time.time()
