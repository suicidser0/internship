#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the Yujin Robot nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.



import roslib; roslib.load_manifest('kobuki_node')
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from kobuki_comms.msg import BumperEvent
from kobuki_comms.msg import Sound

class Test_Translation:
	# constructor
	def __init__(self):
		rospy.init_node("test_translation")
		rospy.loginfo('hello kobuki')
		self._init_params()	# initialize params
		self._init_pubsub()	# initialize pub & sub 	
	
	# initialize params
	def _init_params(self):
		
		self.twist = Twist()

		self.last_odom = 0
		self.odom = 0
		self.odom_count = 0	# Odometry Information
		self.distance = 1	# reference distance

		self.bumper = 0	# bumper 
		self.state = 0 	# bumper state

		self.sound = 0

		self.freq = 5
		self.rate = rospy.Rate(self.freq)

		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0

		self.start = rospy.get_rostime()

		rospy.loginfo("time : %d", self.start.secs)

	def _init_pubsub(self):
		self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist)
		self.odom_sub = rospy.Subscriber("/odom", Odometry, self.OdomInfoCallback)
		self.bumper_event_sub = rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, self.BumperEventCallback)
		self.sound_sub = rospy.Subscriber("/mobile_base/commands/sound", Sound, self.SoundInfoCallback)


	def OdomInfoCallback(self,data):
		#rospy.loginfo("odominfocallback : %.2f", data.pose.pose.position.x)
		self.odom = data.pose.pose.position.x

		if(self.odom_count == 0):
			self.last_odom = self.odom
			self.odom_count = self.odom_count + 1
		
			print "last_odom : %d odom : %d"%(self.last_odom, self.odom)


	def BumperEventCallback(self,data):
		self.state = data.state
		self.bumper = data.bumper

	def SoundInfoCallback(self,data):
		self.sound = data.value


	def ShowOdomInfo(self):

		if math.fabs((self.last_odom+1)-self.odom) <= 0.01:
			self.twist.linear.x = 0.02
			self.cmd_vel_pub.publish(self.twist)

#		elif math.fabs(self.distance - self.odom) <= 0.005:
#			self.twist.linear.x = 0.01
#			self.cmd_vel_pub.publish(self.twist)		

		elif self.odom >= self.last_odom+1 :
			self.twist.linear.x = 0
			self.cmd_vel_pub.publish(self.twist)
		else:
			self.twist.linear.x = 0.2
			self.cmd_vel_pub.publish(self.twist)

		if self.state == 1:
			while not self.state == 0:
				rospy.loginfo("bumper event")
				self.twist.linear.x = 0
				self.cmd_vel_pub.publish(self.twist)
				print('---- bumper event ----')
#				rospy.sleep(3)
			self.state = 0

		if self.odom > 0:
			rospy.loginfo("info_odom : %.4f start_odom : %.4f dist_percent %.4f cmd_vel : %.4f ", self.odom, self.last_odom, (self.odom -self.last_odom)*100, self.twist.linear.x)

	def ShowBumperEventInfo(self):
		if(self.state == BumperEvent.RELEASED):
			state = "released"
		else:
			state = "pressed"
		if(self.bumper == BumperEvent.LEFT):
			bumper = "Left"
		elif(self.bumper == BumperEvent.CENTER):
			bumper = "Center"
		else:
			bumper = "Right"
		rospy.loginfo("%s bumper is %s", bumper, state)

#	def ShowSoundInfo(self):
		


def test_trans_main():
	test_trans_obj = Test_Translation()

	while not rospy.is_shutdown():
		test_trans_obj.ShowOdomInfo() # odom
	
#		test_trans_obj.ShowBumperEventInfo()

#		test_trans.obj.ShowSoundInfo()

	rospy.spin()


if __name__ == '__main__':
	test_trans_main()
