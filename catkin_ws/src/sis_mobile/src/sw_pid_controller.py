#!/usr/bin/env python
import serial
import rospy
import tf

from math import sin, cos, isnan
from Adafruit_MotorHAT import Adafruit_MotorHAT
from simple_pid import PID
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

LX = 0.20/2 # Half of distance between front wheels
LY = 0.14/2 # Half of distance between rear wheels
R  = 0.03 # Tire radius

# Check if the list is in range (low, up)
# Params:
# 	value_list: given value list to check
#	up        : upper bound
# 	low	  : lower bound
# Output:
#	True: if every value in list is in range (low, up)
#	False: otherwise
def in_range(value_list, low, up):
	for i in range(0, len(value_list)):
		if value_list[i] > up or value_list[i] < low:
			return False
	return True

class Car_controller(object):
	def __init__(self):
		self.motorhat = Adafruit_MotorHAT(0x60)
		self.fl = self.motorhat.getMotor(1) # Front left
		self.fr = self.motorhat.getMotor(2) # Front right
		self.rl = self.motorhat.getMotor(3) # Rear left
		self.rr = self.motorhat.getMotor(4) # Rear right
		self.pid_fr = PID(30.0, 20.0, 0.0, sample_time = 0.028)
		self.pid_fl = PID(30.0, 20.0, 0.0, sample_time = 0.028)
		self.pid_rr = PID(30.0, 20.0, 0.0, sample_time = 0.028)
		self.pid_rl = PID(30.0, 20.0, 0.0, sample_time = 0.028)
		self.pid_fr.output_limits = (-255, 255)
		self.pid_fl.putput_limits = (-255, 255)
		self.pid_rr.output_limits = (-255, 255)
		self.pid_rl.output_limits = (-255, 255)
		self.port = rospy.get_param('~port', '/dev/ttyACM0')
		self.pub_tf = rospy.get_param('~pub_tf', False)
		self.ard = serial.Serial(self.port, 57600)
		# Flush serial data
		for i in range(0, 20):
			_ = self.ard.readline()
		# Subscriber and publisher
		self.pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size = 10)
		self.sub_cmd = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size = 1)
		# Service
		self.reset_odom_srv = rospy.Service('reset_wheel_odom', Empty, self.reset_odom)
		if self.pub_tf:
			self.tf_br = tf.TransformBroadcaster()
		self.time = rospy.Time.now()
		rospy.Timer(rospy.Duration(1/35.0), self.read_data) # 35Hz
		# Four wheel angular velocities
		self.w_fr = None 
		self.w_fl = None
		self.w_rr = None
		self.w_rl = None
		# Position variables
		self.x = 0
		self.y = 0
		self.heading = 0
		# Car velocity
		self.v_x = 0 
		self.v_y = 0 
		self.omega = 0
		# Desired velocity
		self.vx_d = 0
		self.vy_d = 0
		self.w_d = 0
		# Odom msg
		self.my_odom = Odometry()
		self.my_odom.header.frame_id = 'odom'
		self.velocity_change = True
		self.shutdown_ = False
		rospy.loginfo("[%s] Initialized" %(rospy.get_name()))
	# reset_wheel_odom callback
	def reset_odom(self, req):
		self.x = 0
		self.y = 0
		self.heading = 0
		print "Reset wheel odom"
		return EmptyResponse()

	# Read data from serial, called by timer
	def read_data(self, event):
		if self.shutdown_:
			self.motor_motion(0, 0, 0, 0)
			return 
		data_str = self.ard.readline()
		data_list = data_str.split()
		try:
			data_list = [float(i)/100. for i in data_list] # Convert to m/s
		except ValueError:
			#print "incorrect data"
			return # incorrect data
		#print data_list
		if len(data_list) != 4:
			#print "incorrect array size"
			return # incorrect array size
		if not in_range(data_list, -5.0, 5.0): # 36RPM -> 3.7699 rad/s
			#print "out of range"
			return # data not in range
		#print data_list
		self.w_fl, self.w_fr, self.w_rl, self.w_rr = data_list
		# dead reckoning
		dt = rospy.Time.now().to_sec() - self.time.to_sec() # time difference
		self.time = rospy.Time.now() # update time
		self.v_x = (self.w_fl + self.w_fr + self.w_rl + self.w_rr) * R / 4
		self.v_y = (-self.w_fl + self.w_fr + self.w_rl - self.w_rr) * R / 4
		self.omega = (-self.w_fl + self.w_fr - self.w_rl + self.w_rr) * R / 4 / (LX + LY)
		self.x = self.x + self.v_x * dt
		self.y = self.y + self.v_y * dt
		self.heading = self.heading + self.omega * dt
		if self.pub_tf:
			self.tf_br.sendTransform((self.x, self.y, 0), 
                                                 (0, 0, sin(self.heading/2), cos(self.heading/2)),
                                                 rospy.Time.now(),
                                                 'car_base',
                                                 'odom')
		# Publish odometry message
		self.my_odom.header.stamp = rospy.Time.now()
		self.my_odom.pose.pose.orientation.z = sin(self.heading/2)
		self.my_odom.pose.pose.orientation.w = cos(self.heading/2)
		self.my_odom.pose.pose.position.x = self.x
		self.my_odom.pose.pose.position.y = self.y
		self.pub_odom.publish(self.my_odom)
		# Send car command
		#print "Error: ", self.pid_fl.error, " ", self.pid_fr.error, " ", \
			#self.pid_rl.error, " ", self.pid_rr.error
		if self.velocity_change == False:
			return 
		if self.vx_d == 0 and self.vy_d == 0 and self.w_d == 0:
			pwm_fl = 0; pwm_fr = 0; pwm_rl = 0; pwm_rr = 0
			self.pid_fl.setpoint = 0
			self.pid_fr.setpoint = 0
			self.pid_rl.setpoint = 0
			self.pid_rr.setpoint = 0
			self.pid_fl.clear_error()
			self.pid_fr.clear_error()
			self.pid_rl.clear_error()
			self.pid_rr.clear_error()
		else:
			# Update setpoint
			self.pid_fl.setpoint = (self.vx_d - self.vy_d - (LX+LY)*self.w_d) / R
			self.pid_fr.setpoint = (self.vx_d + self.vy_d + (LX+LY)*self.w_d) / R
			self.pid_rl.setpoint = (self.vx_d + self.vy_d - (LX+LY)*self.w_d) / R
			self.pid_rr.setpoint = (self.vx_d - self.vy_d + (LX+LY)*self.w_d) / R
			#print self.pid_fl.setpoint, " ", self.pid_fr.setpoint, " ",\
				#self.pid_rl.setpoint, " ", self.pid_rr.setpoint
			# Get PWM values from controller
			# Make sure input is not nan
			if not isnan(self.w_fl) and not isnan(self.w_fr) and not isnan(self.w_rl) \
				and not isnan(self.w_rr):
				pwm_fl = self.pid_fl(self.w_fl)
				pwm_fr = self.pid_fr(self.w_fr)
				pwm_rl = self.pid_rl(self.w_rl)
				pwm_rr = self.pid_rr(self.w_rr)
		self.motor_motion(pwm_fl, pwm_fr, pwm_rl, pwm_rr)
	# sub_cmd callback, get four wheel desired angular velocity and try to complete it through PID 
	# controllers
	def cmd_cb(self, msg):
		if msg.linear.x == self.vx_d and msg.linear.y == self.vy_d and msg.angular.z == self.w_d:
			self.velocity_change = False
		else:
			self.velocity_change = True
		self.vx_d = msg.linear.x
		self.vy_d = msg.linear.y
		self.w_d = msg.angular.z
		
		'''# Reach so vx = vy = omega = 0
		if msg.linear.x == 0 and msg.linear.y == 0 and msg.angular.z == 0:
			print "reach"
			self.pid_fl.auto_mode = False
			self.pid_fr.auto_mode = False
			self.pid_rl.auto_mode = False
			self.pid_rr.auto_mode = False
			self.reach = True
			self.motor_motion(0, 0, 0, 0)
		else:
			self.reach = False
		# Make sure four wheel angular velocity not invalid value
		if not isnan(self.w_fl) and not isnan(self.w_fr) and not isnan(self.w_rl) \
		and not isnan(self.w_rr):
			self.pid_fl.auto_mode = True
			self.pid_fr.auto_mode = True
			self.pid_rl.auto_mode = True
			self.pid_rr.auto_mode = True
			vx_d = msg.linear.x # desired x velocity
			vy_d = msg.linear.y # desired y velocity
			omega_d = msg.angular.z # desired z angular velocity
			self.pid_fl.setpoint = (vx_d - vy_d - (LX+LY)*omega_d) / R
			self.pid_fr.setpoint = (vx_d + vy_d + (LX+LY)*omega_d) / R
			self.pid_rl.setpoint = (vx_d + vy_d - (LX+LY)*omega_d) / R
			self.pid_rr.setpoint = (vx_d - vy_d + (LX+LY)*omega_d) / R
			# Get PWM value from controlleri
			pwm_fl = self.pid_fl(self.w_fl)
			pwm_fr = self.pid_fr(self.w_fr)
			pwm_rl = self.pid_rl(self.w_rl)
			pwm_rr = self.pid_rr(self.w_rr)
			if self.reach is not True:
				self.motor_motion(pwm_fl, pwm_fr, pwm_rl, pwm_rr)'''
	# send command to motors
	# Param:
	#   pwm_fl: PWM value for front left motor 
	#   pwm_fr: PWM value for front right motor
	#   pwm_rl: PWM value for rear left motor
	#   pwm_rr: PWM value for rear right motor
	def motor_motion(self, pwm_fl, pwm_fr, pwm_rl, pwm_rr):
		# FL
		#print "PWM: ", pwm_fl, " ", pwm_fr, " ", pwm_rl, " ", pwm_rr
		if  pwm_fl < 0:
			fl_state = Adafruit_MotorHAT.BACKWARD
			pwm_fl = -pwm_fl
		elif pwm_fl > 0:
			fl_state = Adafruit_MotorHAT.FORWARD
		else: # is 0
			fl_state = Adafruit_MotorHAT.RELEASE
		# FR
		if pwm_fr < 0:
			fr_state = Adafruit_MotorHAT.BACKWARD
			pwm_fr = -pwm_fr
		elif pwm_fr > 0:
			fr_state = Adafruit_MotorHAT.FORWARD
		else: # is 0
			fr_state = Adafruit_MotorHAT.RELEASE
		# RL
		if pwm_rl < 0:
			rl_state = Adafruit_MotorHAT.BACKWARD
			pwm_rl = -pwm_rl
		elif pwm_rl > 0:
			rl_state = Adafruit_MotorHAT.FORWARD
		else: # is 0
			rl_state = Adafruit_MotorHAT.RELEASE
		# RR
		if pwm_rr < 0:
			rr_state = Adafruit_MotorHAT.BACKWARD
			pwm_rr = -pwm_rr
		elif pwm_rr > 0:
			rr_state = Adafruit_MotorHAT.FORWARD
		else: # is 0
			rr_state = Adafruit_MotorHAT.RELEASE
		self.fl.setSpeed(int(pwm_fl))
		self.fr.setSpeed(int(pwm_fr))
		self.rl.setSpeed(int(pwm_rl))
		self.rr.setSpeed(int(pwm_rr))
		self.fl.run(fl_state)
		self.fr.run(fr_state)
		self.rl.run(rl_state)
		self.rr.run(rr_state)
		
	# Shutdown function, call when terminate
	def shutdown(self):
		rospy.loginfo("[%s] Shutting down" %(rospy.get_name()))
		self.sub_cmd.unregister()
		self.shutdown_ = True
		rospy.sleep(1.0)
		self.fl.setSpeed(0)
		self.fr.setSpeed(0)
		self.rl.setSpeed(0)
		self.rr.setSpeed(0)
		self.fl.run(Adafruit_MotorHAT.RELEASE)
		self.fr.run(Adafruit_MotorHAT.RELEASE)
		self.rl.run(Adafruit_MotorHAT.RELEASE)
		self.rr.run(Adafruit_MotorHAT.RELEASE)
		del self.motorhat

if __name__ == '__main__':
	rospy.init_node('pid_controller_sw_node')
	controller_sw = Car_controller()
	rospy.on_shutdown(controller_sw.shutdown)
	rospy.spin()
