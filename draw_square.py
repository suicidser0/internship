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
		self.theta = 0

		self.imu_count = 0
		self.last_imu_data = 0
		self.imu_data = 0

	def ImuInfoCallback(self,data):
		self.imu_data = degrees(data.data)
		
		if(self.imu_count == 0):
			self.last_imu_data = self.imu_data
			self.imu_count = self.imu_count + 1

		#print "angle : %.4f"%self.imu_data

	def OdomInfoCallback(self, data):
		self.odom_data_x = data.pose.pose.position.x
		self.odom_data_y = data.pose.pose.position.y
		#print "odom : %.4f"%odom_data
		if(self.odom_count == 0):
			self.last_odom_data_x = self.odom_data_x
			self.last_odom_data_y = self.odom_data_y
			self.odom_count = self.odom_count + 1


	def Draw_square_kobuki(self):
		twist = Twist()

		if self.goal_state == True:
			goal_x = math.cos(self.theta) + self.last_odom_data_x
			goal_y = math.sin(self.theta) + self.last_odom_data_y
			self.goal_state = False
	
			print "goal_x: %.4f goal_y: %.4f odom_x: %.4f odom_y:%.4f theta: %d"%(goal_x, goal_y, self.odom_data_x, self.odom_data_y, self.theta)

		if math.fabs(goal_x - self.odom_data_x) <= 0.01 and math.fabs(goal_y - self.odom_data_y) <= 0.01:
			if self.theta == 0 or self.theta == 270:
				twist.linear.x = 0.02
				self.cmd_vel_pub.publish(twist)
			else:
				twist.linear.y = 0.02
				self.cmd_vel_pub.publish(twist)

		elif self.odom_data_x >= goal_x :
			twist.linear.x = 0
			self.cmd_vel_pub.publish(twist)
			print "stop"
			rospy.sleep(2)
			#print "last angle : %.4f angle %.4f"%(self.last_imu_data, self.imu_data)
		 	while not  math.fabs(self.last_imu_data - self.imu_data) >= 90:
				print "rotation"
				twist.angular.z = 0.3
				self.cmd_vel_pub.publish(twist)
				print "last angle : %.4f angle %.4f"%(self.last_imu_data, self.imu_data)
			self.odom_count = 0
			self.goal_state = True
			self.theta = self.theta + 90
		else:
			if self.theta == 0 or self.theta == 270:
				twist.linear.x = 0.2
				self.cmd_vel_pub.publish(twist)
			else:
				twist.linear.y = 0.2
				self.cmd_vel_pub.publish(twist)




def draw_square():
	print('---')
	instance = Draw_square()
	while not rospy.is_shutdown():
		instance.Draw_square_kobuki()
	rospy.spin()

if __name__ == '__main__':
	draw_square()

