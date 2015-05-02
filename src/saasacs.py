#!/usr/bin/env python
import roslib
import rospy
import rosbag
import numpy as np
from std_msgs.msg import Int32, String
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
roslib.load_manifest('lab3')
trans_bin = 20
angle_bin = 90
total_angle = 360
total_trans = 700
threshold = 0.1
marker = Marker()
pos = np.zeros((total_trans/trans_bin, total_trans/trans_bin, total_angle / angle_bin))
temp_pos = np.zeros((total_trans/trans_bin, total_trans/trans_bin, total_angle / angle_bin))
tags = np.array([[125, 525],[125, 325],[125, 125],[425, 125],[425, 325],[425, 525]])
def init(): 
	bag = rosbag.Bag('grid.bag')
	counter = 1
	12, 28, 3
	pos[12-1,28-1,3-1] = 1
	try:
		for topic, msg, t in bag.read_messages(topics=['Movements', 'Observations']):
			if topic == 'Movements':
				print counter, topic, msg
				rot1, trans, rot2 = msg.rotation1, msg.translation, msg.rotation2
				rot1 = np.degrees((euler_from_quaternion([rot1.x,rot1.y,rot1.z,rot1.w]))[2])
				if rot1 < 0:
					rot1 = rot1 + 360
				print "Printing ", rot1
				rot2 = np.degrees((euler_from_quaternion([rot2.x,rot2.y,rot2.z,rot2.w]))[2])
				if rot2 < 0:
					rot2 = rot2 + 360
				print "Printing ", rot2
				update_position(rot1, trans, rot2)
			else: 
				print counter, topic, msg
				dist = msg.range
				rot = msg.bearing
				rot = np.degrees((euler_from_quaternion([rot.x, rot.y, rot.z, rot.w]))[2])
				update_position_observation(msg.tagNum, dist, rot)
				# break
			counter = counter + 1
				
	finally:
		bag.close()

def update_position(rot1, trans, rot2):
	global pos, temp_pos, threshold
	temp_pos = pos
	pos = np.zeros((total_trans/trans_bin, total_trans/trans_bin, total_angle / angle_bin))
	norm_value = 0
	total_prob = 0
	for i_t in np.arange(pos.shape[0]):
		for j_t in np.arange(pos.shape[1]):
			for k_t in np.arange(pos.shape[2]):
				if temp_pos[i_t, j_t, k_t] < threshold:
					continue
				for i in np.arange(pos.shape[0]):
					for j in np.arange(pos.shape[1]):
						for k in np.arange(pos.shape[2]):
							rot1_tmp, trans_tmp, rot2_tmp = getmotion(i, j, k, i_t, j_t, k_t)
							rot1_prb = pdf(rot1_tmp, rot1, angle_bin/2)
							trans_prb = pdf(trans_tmp, trans, trans_bin/2)
							rot2_prb = pdf(rot2_tmp, rot2, angle_bin/2) 
							val = temp_pos[i_t, j_t, k_t] * trans_prb * rot1_prb * rot2_prb
							pos[i, j, k] = pos[i, j, k] + val
							total_prob = total_prob + val
				# pos[i, j, k] = total_prob
				# norm_value = norm_value + total_prob
	pos = pos / total_prob
	index = np.argmax(pos)
	index_angle = index % pos.shape[2]
	index = index / pos.shape[2]
	index_y = index % pos.shape[1]
	index = index / pos.shape[1]
	index_x = index % pos.shape[0]
	publish_pose_rviz(index_angle, index_x, index_y)
	print ("Max Index " , index_x, index_y, index_angle)


def update_position_observation(tagnum, trans, rot):
	global pos, temp_pos, threshold
	temp_pos = pos
	pos = np.zeros((total_trans/trans_bin, total_trans/trans_bin, total_angle / angle_bin))
	norm_value = 0
	total_prob = 0
	for i in np.arange(pos.shape[0]):
		for j in np.arange(pos.shape[1]):
			for k in np.arange(pos.shape[2]):
				rot_tmp, trans_tmp = getmotion_observe(i, j, k, tagnum)
				rot_prb = pdf(rot_tmp, rot, angle_bin/2)
				trans_prb = pdf(trans_tmp, trans, trans_bin/2)
				val = temp_pos[i, j, k] * trans_prb * rot_prb
				pos[i, j, k] = pos[i, j, k] + val
				total_prob = total_prob + val
	pos = pos / total_prob
	index = np.argmax(pos)
	index_angle = index % pos.shape[2]
	index = index / pos.shape[2]
	index_y = index % pos.shape[1]
	index = index / pos.shape[1]
	index_x = index % pos.shape[0]
	print ("Max Index " , index_x, index_y, index_angle)
	publish_pose_rviz(index_angle, index_x, index_y)

def publish_pose_rviz(index_angle, index_x, index_y):
	global pos, marker
	shape = Marker.LINE_LIST
	rate = rospy.Rate(10)
	pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
	marker.header.frame_id = "/my_frame"
	marker.header.stamp = rospy.Time.now()
	marker.ns = "motion"
	marker.id = 0
	marker.type = shape
	marker.pose.position.x = index_x
	marker.pose.position.y = index_y
	marker.pose.position.z = 0
	ang = quaternion_from_euler(0,0,index_angle)
	marker.pose.orientation.x = ang[0]
	marker.pose.orientation.y = ang[1]
	marker.pose.orientation.z = ang[2]
	marker.pose.orientation.w = ang[3]
	marker.action = Marker.ADD
	pub.publish(marker)
	rate.sleep()

def getmotion_observe(i, j, k, tagnum):
	global tags
	trans_x = i * trans_bin + trans_bin / 2
	trans_y = j * trans_bin + trans_bin / 2
	rot = k * angle_bin + angle_bin / 2
	
	trans = np.sqrt((trans_x - tags[tagnum,0]) ** 2 + (trans_y - tags[tagnum,1]) ** 2)
	first_angle = 	np.degrees(np.arctan2(tags[tagnum,1]-trans_y, tags[tagnum,0] - trans_x))
	if first_angle < 0:
		first_angle = 360 + first_angle
	rot1 = np.min([np.abs(first_angle - rot), (360 - np.max([rot,first_angle]) + np.min([rot, first_angle]))])
	return rot1, trans

def pdf(x, mean, var):
	val = (1 / (np.sqrt(2 * np.pi) * var)) * np.power(np.e, -1 * (((x - mean)**2)/(2 * var ** 2)))
	return val
def getmotion(i, j, k, i_t, j_t, k_t):
	trans1_x = i * trans_bin + trans_bin / 2
	trans1_y = j * trans_bin + trans_bin / 2
	rot1_1 = k * angle_bin + angle_bin / 2

	trans2_x = i_t * trans_bin + trans_bin / 2
	trans2_y = j_t * trans_bin + trans_bin / 2
	rot2_1 = k_t * angle_bin + angle_bin / 2
	trans = np.sqrt((trans1_x - trans2_x) ** 2 + (trans1_y - trans2_y) ** 2)
	first_angle = 	np.degrees(np.arctan2(trans2_y-trans1_y, trans2_x - trans1_x))
	if first_angle < 0:
		first_angle = 360 + first_angle
	# print "First Angle ", first_angle, " rot1_1: ", rot1_1, " rot2_1: ", rot2_1
	# print "First value ", np.array([np.abs(first_angle - rot1_1), (360 - np.max([rot1_1,first_angle]) + np.min([rot1_1, first_angle]))])
	# print "Second value ", [np.abs(first_angle - rot2_1), (360 - np.max([rot2_1,first_angle]) + np.min([rot2_1, first_angle]))]
	rot1 = np.min([np.abs(first_angle - rot1_1), (360 - np.max([rot1_1,first_angle]) + np.min([rot1_1, first_angle]))])
	rot2 = np.min([np.abs(first_angle - rot2_1), (360 - np.max([rot2_1,first_angle]) + np.min([rot2_1, first_angle]))])
	return rot1, trans, rot2


if __name__ == '__main__':
	try:
		rospy.init_node('lab3')
		init()
	except rospy.ROSInterruptException:
		pass
