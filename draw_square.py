#!/usr/bin/env python

import roslib; roslib.load_manifest('minsu_apps')
import rospy
import math
from math import degrees
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class Draw_square:
	def __init__(self):
		rospy.init_node("draw_square_kobuki")
		self._init_params()
		self._init_pubsub()

	def _init_pubsub(self):
		self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist)
		self.odom_sub = rospy.Subscriber("odom", Odometry, self.OdomInfoCallback)
		self.sub_imu = rospy.Subscriber("angle", Float64, self.ImuInfoCallback)

	def _init_params(self):
		self.odom_count = 0
		self.last_odom_data_x = 0
		self.last_odom_data_y = 0
		self.odom_data_x = 0
		self.odom_data_y = 0
		self.goal_state = True

#		self.theta = 0

		#global alpha
		self.alpha = 0

		self.imu_count = 0
		self.last_imu_data = 0
		self.imu_data = 0

	def ImuInfoCallback(self,data):
		self.imu_data = degrees(data.data)
		
		if(self.imu_count == 0):
			print " ----- update imu callback ----- "
			self.last_imu_data = self.imu_data
			self.imu_count = self.imu_count + 1

		#print "angle : %.4f"%self.imu_data

	def OdomInfoCallback(self, data):
		self.odom_data_x = data.pose.pose.position.x
		self.odom_data_y = data.pose.pose.position.y
		#print "odom : %.4f"%odom_data
		if(self.odom_count == 0):
			print " ----- update odom callback ----- "
			self.last_odom_data_x = self.odom_data_x
			self.last_odom_data_y = self.odom_data_y
			self.odom_count = self.odom_count + 1

	def Turning_State(self):
		while not math.fabs(self.last_imu_data - self.imu_data) >= 90 and not rospy.is_shutdown():
			print "rotation"
			twist.angular.z = 0.25
			self.cmd_vel_pub.publish(twist)
			print "last angle: %.4f angle: %.4f"%(self.last_imu_data, self.imu_data)
		self.odom_count = 0
		self.imu_count = 0
		self.goal_state = True


	def Draw_square_kobuki(self):
		global goal_x
		global goal_y
		global mod
		global robot_state
		twist = Twist()
		global twist

		if self.goal_state == True:
			goal_x = math.cos((math.pi/2)*self.alpha) + self.last_odom_data_x
			goal_y = math.sin((math.pi/2)*self.alpha) + self.last_odom_data_y
			print "--- set goal position ---"
			print "goal_x: %.4f goal_y: %.4f odom_x: %.4f odom_y:%.4f theta: %d"%(goal_x, goal_y, self.odom_data_x, self.odom_data_y, self.alpha)
			print "-------------------------"
			rospy.sleep(5)
			robot_state = self.alpha%2
			if self.alpha%2 == 0:
				mod = 1
			else:
				mod = -1 
			self.alpha = self.alpha + 1
			self.goal_state = False

		
		if robot_state == 0:
			if math.fabs(goal_x - self.odom_data_x) <= 0.1:
				twist.linear.x = 0.02
				if math.fabs(goal_x - self.odom_data_x) <= 0.05:
					print "stop x axis"
					twist.linear.x = 0
					rospy.sleep(5)
					self.Turning_State()
			else:
				twist.linear.x = 0.2
			self.cmd_vel_pub.publish(twist)

		elif robot_state == 1:
			if math.fabs(goal_y - self.odom_data_y) <= 0.1:
				twist.linear.x = 0.02
				if math.fabs(goal_y - self.odom_data_y) <= 0.05:
					print "stop y axis"
					twist.linear.x = 0
					rospy.sleep(5)
					self.Turning_State()
			else:
				twist.linear.x = 0.2
			self.cmd_vel_pub.publish(twist)

		print "goal_x: %.4f goal_y: %.4f odom_x: %.4f odom_y:%.4f state: %d"%(goal_x, goal_y, self.odom_data_x, self.odom_data_y, robot_state)

def draw_square():
	print('---')
	instance = Draw_square()

	rospy.sleep(1)

	while not rospy.is_shutdown():
		instance.Draw_square_kobuki()
	rospy.spin()

if __name__ == '__main__':
	draw_square()

