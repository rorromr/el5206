#!/usr/bin/env python
# Autor> Ruben Claveria Vega
# Curso> EL5206 Laboratorio de Inteligencia Computacional y Robotica

import robot_utilities
import time
import rospy
import math
import numpy
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry     
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

class Controller:
	def __init__(self):

		# inicializa nodo ROS
		rospy.init_node('Simulation');

		# robotID = ID del robot que se desea controlar
		# si hay solo un robot, robotID debe ser ""
		# si hay mas de un robot, robotID debe ser de la forma "/robot_0", "/robot_1", etc.
		robotID = ""

		# Posicion y orientacion iniciales.
		# IMPORTANTE: deben ser consistentes con la del archivo .world
		init_x = -2.0
		init_y = -2.0
		init_angle = 90.0

		# creacion de un objeto de tipo Robot
		R = robot_utilities.Robot(robotID, init_x, init_y, init_angle)

		# Subscripcion a los topicos de interes: odometria (Odometry) y sensor de distancia (LaserScan)
		rospy.Subscriber(robotID+'/odom', Odometry, R.odom_callback)
		rospy.Subscriber(robotID+'/base_scan', LaserScan, R.ranger_callback)
		# Observacion: Las funciones de callback se ejecutan cada vez que el programa recibe informacion 
		# desde un topico. Se utilizan, generalmente, para actualizar datos.
		# Pueden utilizarse funciones de callback programadas por el alumno, 
		# diferentes a las provistas por la clase Robot

		# Se recomienda que este ciclo vaya antes de todas las instrucciones de control
		# Sirve para asegurar que el Robot recibio su primera lectura de sensor
		while not rospy.is_shutdown() and len(R.distances)==0:
			R.rate.sleep()

		# Ciclo de ejemplo. 
		# Si se va a controlar el robot con una funcion de robot_utilities en vez de con el ciclo:
		# opcion 1: cambiar True por False en la condicion del while
		# opcion 2: comentar el bloque de codigo
		x = -3.7
		y = 1.97
		th = 5.39*math.pi/180.0
		while not rospy.is_shutdown():
			# Define velocidad lineal [metros/segundo] del robot durante la iteracion
			R.cmd.linear.x = 0.2
			
			# Velocidad angular [radianes/segundo]; rango aceptable: [-pi/2, pi/2 ], aprox. [-1.57, 1.57]
			R.cmd.angular.z = 0.0 

			# indica a Stage la velocidad que el robot tendra durante un ciclo
			R.cmd_pub.publish(R.cmd)
			# Ejemplo de como rescatar datos captados por los sensores
			# En este caso solo se imprimen, pero la idea es utilizarlos en decisiones de control
			R.move_circ(2,math.pi)

		# Funciones de ejemplo (uncomment para usar): 
		#R.nSteps(1500,1500)
		#R.nSteps(1500,900)
		#R.nSteps(900,1500)
		#R.moveTill(math.pi*5.0/2, 0.5, 0.1)
        
if __name__ == "__main__": Controller()
