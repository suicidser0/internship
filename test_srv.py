#!/usr/bin/env python

import roslib; roslib.load_manifest('begin_py')
import rospy
import sys 
from begin_py.srv import *

class service:

	def __init__(self):
		rospy.init_node('sub_two_ints_server')	
		self._init_srvcli()
		req = SubTwoInts()

	def _init_srvcli(self):
		rospy.Service('sub_two_ints', SubTwoInts, self.handle_sub_two_ints)

	def handle_sub_two_ints(self, req):
		print "returning %s - %s = %s"%(req.a, req.b, (req.a - req.b))
		return SubTwoIntsResponse(req.a - req.b)

def service_main():
	print "ready to sub two ints"
	service_obj = service()
	rospy. spin()

if __name__ == "__main__":
	service_main()

