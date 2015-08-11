"""Kalman filter wrapper class.
Note about units:
	time -- sec
	linear distance -- mm
	linear velocity -- mm/sec
	angle -- radians
	angular velocity -- radians/sec
"""
import cv
from math import cos, sin, pi
import numpy
import sys
import os

printme = False

class PsoKalman:
	def __init__(self):
		self.__kalman = cv.CreateKalman(8, 5, 4)
	
		state = cv.CreateMat(8, 1, cv.CV_32FC1) #x, x', y, y', z, z', theta, theta'
		process_noise = cv.CreateMat(8, 1, cv.CV_32FC1)
		
		cv.SetIdentity(self.__kalman.transition_matrix, cv.RealScalar(1))
		cv.Zero(self.__kalman.control_matrix)
		cv.Zero(self.__kalman.measurement_matrix)
		self.__kalman.measurement_matrix[0, 1] = 1
		self.__kalman.measurement_matrix[1, 3] = 1
		self.__kalman.measurement_matrix[2, 4] = 1
		self.__kalman.measurement_matrix[3, 5] = 1
		self.__kalman.measurement_matrix[4, 6] = 1
		self.__theta_offset = 0
		cv.SetIdentity(self.__kalman.process_noise_cov, cv.RealScalar(.01))
		cv.SetIdentity(self.__kalman.measurement_noise_cov, cv.RealScalar(0))
		cv.SetIdentity(self.__kalman.error_cov_post, cv.RealScalar(1))
		
	def reset(self):
		"""Reset the state to all zeros (also sets the offset angle to offset from
		the drone's internal heading"""
		self.__theta_offset += self.__kalman.state_post[6, 0]
		cv.Zero(self.__kalman.state_post)
		cv.Zero(self.__kalman.state_pre)
		cv.SetIdentity(self.__kalman.error_cov_post, cv.RealScalar(1))
	
	def update(self, measurement, command, dt):
		"""Perform a Kalman filter update and return the state as a 
		cvMat(8, 1, cv.CV_32FC1). Note that this function calls heading
		theta. (In Parrot's code it is psi.)
			measurement -- cvMat x', y', z, z', heading
			command -- cvMat pitch, roll, gaz, yaw
			dt -- Time step
		"""
			
		theta = self.__kalman.state_post[6, 0]
		
		# Create the state transition matrix based on time interval
		self.__kalman.transition_matrix[0, 1] = dt;
		self.__kalman.transition_matrix[2, 3] = dt;
		self.__kalman.transition_matrix[4, 5] = dt;
		self.__kalman.transition_matrix[6, 7] = dt;
		
		#print "A:"
		#print numpy.asarray(self.__kalman.transition_matrix)
		
		#Create the control matrix based on the heading
		self.__kalman.control_matrix[1, 0] = cos(theta)
		self.__kalman.control_matrix[1, 1] = sin(theta)
		self.__kalman.control_matrix[3, 0] = -sin(theta)
		self.__kalman.control_matrix[3, 1] = cos(theta)
		
		#print "B:"
		#print numpy.asarray(self.__kalman.control_matrix)
		if printme:
			print "H:"
			print numpy.asarray(self.__kalman.measurement_matrix)
		
		#Convert measurement from drone space to world space
		if printme:
			print "Theta: ",theta
		temp_x = - measurement[0, 0] * sin(theta) + measurement[1, 0] * cos(theta)
		temp_y = measurement[0, 0] * cos(theta) - measurement[1, 0] * sin(theta)
		measurement[0, 0] = temp_x
		measurement[1, 0] = temp_y
		measurement[4, 0] -= self.__theta_offset
		if printme:
			print "Theta Offset: ",self.__theta_offset
			print "Measurement:"
			print numpy.asarray(measurement)
			#Do feedforward stuff on B to convert from pitch/roll angles to velocities
			print "Command:"
			print numpy.asarray(command)
		
		#print "B*u:"
		#temp = cv.CreateMat(8, 1, cv.CV_32FC1)
		#cv.MatMul(self.__kalman.control_matrix, command, temp)
		#print numpy.asarray(temp)
				
		cv.KalmanPredict(self.__kalman, command)
		#print "Prediction:"
		#print numpy.asarray(self.__kalman.state_pre)
		
		state = cv.KalmanCorrect(self.__kalman, measurement)
		#print "Corrected:"
		#print numpy.asarray(self.__kalman.state_post)
		
		#print "P:"
		#print numpy.asarray(self.__kalman.error_cov_post)
		
		#print "K:"
		#print numpy.asarray(self.__kalman.gain)
		return state

if __name__ == "__main__":
	code = -1L

	dt = .1
	kalman = PsoKalman()
	measurement = cv.CreateMat(5, 1, cv.CV_32FC1) #x', y', z, z', theta
	command = cv.CreateMat(4, 1, cv.CV_32FC1) #pitch, roll, gaz, yaw
	while True:
		os.system("clear")
		measurement[0, 0] = 1
		measurement[1, 0] = 0
		measurement[4, 0] = pi/4
		cv.Zero(command)
		kalman.update(measurement, command, dt)
		code = cv.WaitKey(100)
		if code != -1:
			break
