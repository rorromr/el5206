#!/usr/bin/env python
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

# En lugar de 'testpy', deberia ir el nombre del paquete creado.
import time
import rospy
import math
import numpy
import random
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 


# Si solo hay un robot en Stage, robotTag = ""
# Si hay mas de un robot en Stage, robotTag = "/robot_0", "/robot_1", etc.
class Robot:
	def __init__(self, robotID, init_x, init_y, init_angle):

		# posicion y angulo
		self.pose_x=init_x
		self.pose_y=init_y
		self.angle = init_angle

		# posicion y angulo iniciales
		self.init_x=init_x
		self.init_y=init_y
		self.angle = init_angle

		# cambio en la posicion en la ultima iteracion
		self.delta_x=0.0
		self.delta_y=0.0
		
		# distancia recorrida
		self.perim = 0.0

		# distancias medidas por el laser
		self.distances = []

		# arreglo con los angulos a los que corresponde cada muestra del sensor
		self.laser_angles = []

		# variable auxiliar
		self.laser_angles1 = []
		
		# matriz de parametros de ruido odometrico, de distancia (ranger) y angular (ranger)
		# odomNoise_ON/OFF={0.0, 1.0}		odomNoise_mu		odomNoise_sigma
		# rangerNoise_ON/OFF={0.0, 1.0}		rangerNoise_mu		rangerNoise_sigma
		# angularNoise_ON/OFF={0.0, 1.0}	angularNoise_mu		angularNoise_sigma
		self.noise = [[0,0,0],[0,0,0],[0,0,0]]

		# indica si se ha recibido la primera lectura de sensor
		self.distances_OK = False

		# comunicacion con ROS
		self.cmd_pub=rospy.Publisher(robotID+'/cmd_vel', Twist)
		self.cmd = Twist()
		self.rate = rospy.Rate(20.0)


	#########################################
	############### FUNCIONES ###############
	#########################################

	# El robot se mueve hasta recorrer una distancia igual a perimeter 
	def moveTill(self, perimeter, lin_vel, ang_vel):

		while not rospy.is_shutdown() and self.perim<=perimeter:
			self.cmd.linear.x = lin_vel
			self.cmd.angular.z = ang_vel
			self.cmd_pub.publish(self.cmd)
			self.rate.sleep()

	# El robot se mueve (nL, nR) pulsos de encoder
	def nSteps(self, nL, nR):
		if nL == 0 and nR == 0:
			return None
		P = 150.0
		R = 0.09
		N = 500
		t_sim = (1.0*max(abs(nL),(nR)))/P
		trcs = 0.4
		aux = (1.0*min(abs(nL),abs(nR)))/(1.0*max(abs(nR),abs(nL))) 
		if abs(nL)>=abs(nR):
			VL = math.copysign(1.0,nL) * P * 2.0 * math.pi * R / N
			VR = math.copysign(1.0,nR) * abs(VL) * aux
		else:
			VR = math.copysign(1.0,nR) * P * 2.0 * math.pi * R / N
			VL = math.copysign(1.0,nL) * abs(VR) * aux
	
		v_lin = (VL + VR)/2.0
		v_ang = (VR - VL)/trcs

		tm = rospy.get_rostime()
		while tm.secs == 0 and tm.nsecs == 0:
			tm = rospy.get_rostime()

		while not rospy.is_shutdown():
			cur = rospy.get_rostime()
			self.cmd.linear.x = v_lin
			self.cmd.angular.z = v_ang
			self.cmd_pub.publish(self.cmd)	
			self.rate.sleep()
			if not (cur - tm) < rospy.Duration(t_sim):
				break

	# Muestra distancia total recorrida
	def show_distance(self):
		print 'Distancia recorrida [m]: ' + str(self.perim)

	# Resetea distancia recorrida
	def reset_perim(self):
		self.perim = 0.0

	# Transforma angulos leidos en odom_callback en angulos en el rango [0, 360)
	# Uso: self.angle_tf(self.alpha, self.alpha_w); R.angle_tf(R.alpha, R.alpha_w)
	def angle_tf(self, alpha_z, alpha_w):
		if alpha_z>=0:
			return alpha_z
		else:
			return 360 + alpha_z

	# Funcion de callback: se debe ejecutar cada vez que el subscriptor recibe informacion (/odom)
	# Actualiza posicion, camino recorrido, orientacion en funcion de los datos odometricos recibidos
	def odom_callback(self, data):
		self.delta_x = data.pose.pose.position.x - self.pose_x
		self.delta_y = data.pose.pose.position.y - self.pose_y
		self.pose_x = data.pose.pose.position.x + self.noise[0][0]*random.gauss(self.noise[0][1],self.noise[0][2])
		self.pose_y = data.pose.pose.position.y + self.noise[0][0]*random.gauss(self.noise[0][1],self.noise[0][2])
		alpha_z = numpy.arcsin(data.pose.pose.orientation.z)*360/math.pi
		alpha_w = numpy.arccos(data.pose.pose.orientation.w)*360/math.pi
		self.angle = self.angle_tf(alpha_z, alpha_w)
		self.perim = self.perim + (self.delta_x**2 + self.delta_y**2)**0.5

	# Funcion de callback para (/base_scan)
	# Actualiza las distancias medidas
	def ranger_callback(self, data):

		if self.distances_OK == False:
			self.distances = data.ranges
			for i in range(0,len(data.ranges)):
				self.laser_angles1.append(data.angle_min+i*data.angle_increment)
			self.laser_angles = self.laser_angles1
			if len(self.distances)>0:
				self.distances_OK = True

			else:
				return None

		if self.noise[1][0] == 1:
			aux_noise_r = [random.gauss(self.noise[1][1],self.noise[1][2]) for i in xrange(len(data.ranges))]
			self.distances = map(sum, zip(data.ranges, aux_noise_r))
		else:
			self.distances = data.ranges
		if self.noise[2][0] == 1:
			aux_noise_a = [random.gauss(self.noise[2][1],self.noise[2][2]) for i in xrange(len(data.ranges))]
			self.laser_angles = map(sum, zip(self.laser_angles1, aux_noise_a))
		else:			
			self.laser_angles = self.laser_angles1

	def calc_fk(self, dnr, dnl, thetaA):
		R_N = 1.82914e-4
		W = 0.406476
		# Wheel distance
		dSl = 2 * math.pi * R_N * dnl
		dSr = 2 * math.pi * R_N * dnr
		#
		dS = (dSl + dSr)/2
		dtheta = (dSl - dSr)/W
		dalpha = dtheta
		#
		dx = (- dS * math.sqrt(2*(1-math.cos(dalpha))) )/dalpha * math.sin(thetaA - math.asin(math.sin(dalpha)/math.sqrt(2*(1-math.cos(dalpha)))))
		dy = (  dS * math.sqrt(2*(1-math.cos(dalpha))) )/dalpha * math.cos(thetaA - math.asin(math.sin(dalpha)/math.sqrt(2*(1-math.cos(dalpha)))))
		return dx,dy,dtheta

	def param_w(self):
		step = [60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60]
		for s in step:
			x0 = self.angle*math.pi/180.0
			self.nSteps(-s,s)
			estimated = (4*math.pi*1/5467.042*s)/(self.angle*math.pi/180.0-x0)
			print '%.4e' % estimated

	def param_rn(self):
		step = [60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60,60]
		for s in step:
			x0 = self.perim
			self.nSteps(s,s)
			estimated = 1/((2*math.pi*s)/(self.perim-x0))
			print '%.4e' % estimated

	def move_rect(self, x, y):
		R_N = 1.82914e-4
		W = 0.406476
		dx = x - self.pose_x
		dy = y - self.pose_y
		ds = math.sqrt(dx**2 + dy**2)
		theta = math.atan2(dy,dx)
		if theta < 0:
			theta = 2*math.pi + theta
		dtheta = (theta - self.angle*math.pi/180)
		if dtheta < 0:
			dtheta = 2*math.pi + dtheta
		# Angular movement
		n_angular = dtheta*W/(4*math.pi*R_N)
		self.nSteps(-n_angular, n_angular)
		# Linear movement
		n_linear = ds/(2*math.pi*R_N)
		self.nSteps(n_linear,n_linear)

	def move_circ(self, R, dalpha):
		R_N = 1.82914e-4
		W = 0.406476
		ds_L = dalpha * (R-W/2)
		ds_R = dalpha * (R+W/2)

		n_L = ds_L/(2*math.pi*R_N)
		n_R = ds_R/(2*math.pi*R_N)
		self.nSteps(n_L,n_R)

