#!/usr/bin/env python

import robot_utilities
import time
import rospy
import math
import numpy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from random import randint

class Line(object):
	def __init__(self, p1, p2):
		self.m = (p1[1]-p2[1])/(p1[0]-p2[0])
		self.b = p1[1] - self.m*p1[0]
	
	def distance(self, p):
		return math.fabs(self.m*p[0]-p[1]+self.b)/math.sqrt(self.m**2+1)

	def __str__(self):
		return "(m,b)=%.3f,%.3f" % (self.m, self.b)

def ransac_line(p, tolerance=0.1, L = 5, N = 30):
	for k in range(L):
		i = randint(0,len(p))
		j = randint(0,len(p))
		while (i==j): j = randint(0,len(p))

		try:
			line = Line(p[i,:],p[j,:])
		except Exception, e:
			continue

		K = []
		for i, point in enumerate(p):
			if line.distance(point) < tolerance:
				K.append(i)
		if len(K) > N:
			print len(K)
			return K
	return []

class Controller:
	def __init__(self):
		rospy.init_node('ransac');
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = -2.0
		init_y = -2.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		self.robot = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		rospy.Subscriber(robotID+'/odom', Odometry, self.robot.odom_callback)
		rospy.Subscriber(robotID+'/base_scan', LaserScan, self.robot.ranger_callback)

		while not rospy.is_shutdown() and len(self.robot.distances)==0:
			self.robot.rate.sleep()

	def spin(self):
		p = numpy.ones((len(self.robot.distances),3))
		theta = self.robot.angle*math.pi/180.0
		# Rotation matrix
		m = numpy.array([
			[math.cos(theta), -math.sin(theta), self.robot.pose_x],
			[math.sin(theta),  math.cos(theta), self.robot.pose_y],
			[0.0,                          0.0,               1.0]])
		# Calc relative cartesian
		for i,r in enumerate(self.robot.distances):
			p[i,0] = r*math.cos(self.robot.laser_angles[i])
			p[i,1] = r*math.sin(self.robot.laser_angles[i])
		# Apply transform
		pp = numpy.dot(p, m.T)
		# Plot
		plt.plot(pp[:,0], pp[:,1], 'ro')
		plt.axis([-7, 7, -7, 7])
		

		k = ransac_line(pp)
		if k:
			plt.plot(pp[k,0], pp[k,1], 'bo')
		plt.show(block=True)
		# while not rospy.is_shutdown():
		# 	pass
		# pass
        
if __name__ == "__main__":
	c = Controller()
	c.spin()
