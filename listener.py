#!/usr/bin/env python
import rospy
import tf
if __name__ == '__main__':
	""" A simple example of tf listener """
	rospy.init_node('example_tf_listener')
	listener = tf.TransformListener()
	rate = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('cafe_table', 'base', rospy.Time(0)) # left_gripper, base
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		Translation = [trans[0] , trans[1] , trans[2]]
		Quaternion = [rot[0] , rot[1] , rot[2], rot[3]]
		Angles = tf.transformations.euler_from_quaternion([rot[0] , rot[1] , rot[2], rot[3]])
		print("Translation: ", Translation)
		print("Quaternion: ", Quaternion)
		print("Angles: ", Angles)
		print("")
		rate.sleep()
