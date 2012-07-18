#!/usr/bin/env python
 
# making class in python

import roslib; roslib.load_manifest("begin_py")
import rospy
 
class Test:
	def __init__(self):
		print('---')
     	rospy.init_node("test_node")

def Test_main():
   		test_obj = Test()
   		rospy.loginfo("hello")
 
if __name__ == '__main__':
    Test_main()
