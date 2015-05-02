#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as np
from std_msgs.msg import Int32, String
roslib.load_manifest('lab3')

def init():
	bag = rosbag.Bag('grid.bag')
	counter = 1
	try:
		for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
			if topic == 'Movements':
				print counter, topic, msg
			else: 
				print counter, topic, msg
				break
			counter = counter + 1

	finally:
		bag.close()

if __name__ == '__main__':
	try:
		rospy.init_node('lab3', anonymous=True)
		init()
	except rospy.ROSInterruptException:
		pass
