#!/usr/bin/env python

'''
  PID controller for differential-drive robot 
  Two main things to do:
   * Read data from Arduino (in format v_l v_r theta), use this data to do dead-reckoning
     , publish odom topic and move motors -> called by timer with 50Hz
   * Subscribe to twist topic, update desired linear and angular velocity -> called by cb
'''

import serial
import rospy
import tf
import os

from math import sin, cos, isnan
from Adafruit_MotorHAT import Adafruit_MotorHAT
from simple_pid import PID
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from std_srvs.srv import Empty, EmptyResponse

WIDTH = 0.172 # Two wheels distance, in meter
RADIUS = 0.032 # Tire radius, in meter

# Check if the list is in range (low, up)
# Params:
# 	value_list:  given value list
#	low:         lower bound
# 	up:          upper bound
# Output:
#   True: if every value in list in range (low, up)
#   False: otherwise
def in_range(value_list, low, up):
	for i in range(0, len(value_list)):
		if value_list[i] > up or value_list[i] < low:
			return False
	return True

class Car_controller(object):
	def __init__(self):
		print os.getcwd()
		self.motorhat = Adafruit_MotorHAT(0x60)
		self.left_motor = self.motorhat.getMotor(1)
		self.right_motor = self.motorhat.getMotor(2)
		self.pid_r = PID(3000.0, 500.0, 0.0, sample_time = 0.02) # P/I/D for right wheel
		self.pid_l = PID(3000.0, 600.0, 0.0, sample_time = 0.02) # P/I/D for left wheel
		self.pid_d = PID(1000.0, 200.0, 20.0, sample_time = 0.02) 
		# P/I/D for two wheel velocities difference
		self.pid_r.output_limits = (-255, 255)
		self.pid_l.output_limits = (-255, 255) # PWM limit
		self.port = rospy.get_param("~port", '/dev/ttyACM0') # Arduino port from parameter server
		self.pub_tf = rospy.get_param("~pub_tf", False) # If true, broadcasr transform from
		# odom to car_base
		self.ard = serial.Serial(self.port, 57600)
		# Flush serial data
		for i in range(0, 20):
			_ = self.ard.readline()
		# Subscriber and publisher
		self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size = 10)
		self.sub_cmd  = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb,  queue_size = 1)
		if self.pub_tf:
			self.tf_br = tf.TransformBroadcaster()
		# Service
		self.reset_odom = rospy.Service('reset_wheel_odom', Empty, self.reset_odom)
		rospy.Timer(rospy.Duration(1/70.), self.read_data) # 70Hz
		# Left and right wheel velocities
		self.v_r = None
		self.v_l  = None
		# Position variables
		self.heading = 0
		self.x = 0
		self.y = 0
		# Desired velocities
		self.v_d = 0
		self.w_d = 0
		self.time = rospy.Time.now()
		self.shutdown_ = False
		rospy.loginfo("[%s] Initialized"  %(rospy.get_name()))
	# Service callback: call Arduino to reset odometry data
	def reset_odom(self, req):
		self.x = 0
		self.y = 0
		self.heading = 0
		self.ard.write("c".encode()) # Write to the serial to make Arduino clear theta param
		print "Reset wheel odom" 
		return EmptyResponse()
	# Read data from serial and send car command, called by timer
	def read_data(self, event):
		if self.shutdown_:
			return
		data_str = self.ard.readline()
		data_list = data_str.split()
		#print data_list
		try:
			data_list = [float(i)/100 for i in data_list] # Cnnvert cm/s to m/s
		except ValueError:
			#print "incorrect data"
			return # incorrect data
		if len(data_list) != 3:
			#print "incorrect array size"
			return # incorrect array size
		# We use 36 RPM motor -> 36/60*2*pi*0.032 = 0.12 m/s
		# Take two times as limitation
		if not in_range(data_list[0:2], -0.24, 0.24):
			#print "out of range"
			return # data not in range
		self.v_r, self.v_l, heading = data_list
		# dead reckoning
		dt = rospy.Time.now().to_sec() - self.time.to_sec() # time difference
		self.time = rospy.Time.now() # update time
		v = (self.v_r + self.v_l) / 2
		omega = (self.v_r - self.v_l) / WIDTH
		sth = sin(self.heading)
		cth = cos(self.heading)
		dth = omega * dt
		if self.v_r != self.v_l:
			R = (self.v_r + self.v_l) / (self.v_r - self.v_l) * WIDTH / 2
			A = cos(dth) -1
			B = sin(dth)
			self.x += R*(sth*A  + cth*B)
			self.y += R*(cth*-A + sth*B)
		else: # go straight
			self.x += v*dt*cth
			self.y += v*dt*sth
		self.heading = heading
		if self.pub_tf:
			# Broadcast transform from odom to car_base
			self.tf_br.sendTransform((self.x, self.y, 0),
			                         (0, 0, sin(self.heading/2), cos(self.heading/2)),
			                         rospy.Time.now(),
			                         'car_base',
			                         'odom') 
		# Publish odometry message
		odom = Odometry()
		odom.header.frame_id = 'odom'
		odom.header.stamp = rospy.Time.now()
		odom.pose.pose.orientation.z = sin(self.heading/2)
		odom.pose.pose.orientation.w = cos(self.heading/2)
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.covariance[0] = 0.1 # X
		odom.pose.covariance[7] = 0.1 # Y
		odom.pose.covariance[35] = 0.4 # RZ
		self.pub_odom.publish(odom)
		# Execute motor command
		v_d_r = self.v_d + WIDTH/2*self.w_d 
		v_d_l = self.v_d - WIDTH/2*self.w_d
		self.pid_r.setpoint = v_d_r
		self.pid_l.setpoint = v_d_l
		self.pid_d.setpoint = self.w_d
		# Get pwm value from controller
		if self.v_d == 0 and self.w_d == 0:
			pwm_r = pwm_l = pwm_d = 0
                        self.pid_r.clear_error()
                        self.pid_l.clear_error()
			self.pid_d.clear_error()
		elif self.w_d == 0:
			pwm_r = self.pid_r(self.v_r)
			pwm_l = self.pid_l(self.v_l)
			pwm_d = self.pid_d((self.v_r - self.v_l))
		else:
			pwm_r = self.pid_r(self.v_r)
			pwm_l = self.pid_l(self.v_l)
			pwm_d = 0
		#print self.pid_r.error, self.pid_l.error, pwm_r, pwm_l
		self.motor_motion(pwm_r + pwm_d, pwm_l - pwm_d)
	# sub_cmd callback, get desired linear and angular velocity
	def cmd_cb(self, msg):
		self.v_d = msg.linear.x
		self.w_d = msg.angular.z
		
	# Send command to motors
	# Param:
	#   pwm_r: right motor PWM value
	#   pwm_l: left motor PWM value
	def motor_motion(self, pwm_r, pwm_l):
		if pwm_r < 0:
			right_state = Adafruit_MotorHAT.BACKWARD
			pwm_r = -pwm_r
		elif pwm_r > 0:
			right_state = Adafruit_MotorHAT.FORWARD
		else:
			right_state = Adafruit_MotorHAT.RELEASE
		if pwm_l < 0:
			left_state  = Adafruit_MotorHAT.BACKWARD
			pwm_l = -pwm_l
		elif pwm_l > 0:
			left_state = Adafruit_MotorHAT.FORWARD
		else:
			left_state = Adafruit_MotorHAT.RELEASE
		self.right_motor.setSpeed(int(pwm_r))
		self.left_motor.setSpeed(int(pwm_l))
		self.right_motor.run(right_state)
		self.left_motor.run(left_state)
		#if pwm_r == 0 and pwm_l == 0:
			#rospy.sleep(1.0)
	# Shutdown function, call when terminate
	def __del__(self):
		self.shutdown_ = True
		self.right_motor.setSpeed(0)
		self.left_motor.setSpeed(0)
		self.right_motor.run(Adafruit_MotorHAT.RELEASE)
		self.left_motor.run(Adafruit_MotorHAT.RELEASE)
		rospy.sleep(3.0)
		del self.motorhat
	def shutdown(self):
		rospy.loginfo("[%s] Shutting down" %(rospy.get_name()))
		self.sub_cmd.unregister()
		#self.right_motor.setSpeed(0)
		#self.left_motor.setSpeed(0)
		#rospy.sleep(3.0)
		self.__del__()
		

if __name__ == '__main__':
	rospy.init_node('pid_controller_node')
	controller = Car_controller()
	rospy.on_shutdown(controller.shutdown)
	rospy.spin()
	del controller
