#!/usr/bin/env python

import serial
import time
import rospy
from motor_control.msg import motor_pwm


class pwm_control(object):
	def __init__(self):
		self.COM_PORT = "/dev/ttyACM0"
		self.BAUD_RATES = 9600
		self.ser = serial.Serial(self.COM_PORT, self.BAUD_RATES)
		self.dir_l = 1
		self.dir_r = 1
		self.pwm_l = 0
		self.pwm_r = 0

		image_sub = rospy.Subscriber('/vimo_velocity_controller/pwm', motor_pwm, self.pwm_callback)

	def pwm_callback(self, pwm_msg):
		if pwm_msg.pwm_l < 0 :
			self.dir_l = 0
		else:
			self.dir_l = 1
		if pwm_msg.pwm_r < 0 :
			self.dir_r = 0
		else:
			self.dir_r = 1
		self.pwm_l = abs(pwm_msg.pwm_l)
		self.pwm_r = abs(pwm_msg.pwm_r)
		command = "#" + str(self.pwm_l) + "%" + str(self.pwm_r) + "%" + \
				  str(self.dir_l) + "%" + str(self.dir_r) + "%"
		self.ser.write(command+'\n')
		self.ser.close()

	def onShutdown(self):
		self.pwm.set_pwm(self.l_port, 0, 0)
		self.pwm.set_pwm(self.r_port, 0, 0)
		rospy.loginfo("Shutdown.")

if __name__ == '__main__': 
	rospy.init_node('pwm_control',anonymous=False)
	pwm_control = pwm_control()
	rospy.on_shutdown(pwm_control.onShutdown)
	rospy.spin()
		
