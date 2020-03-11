#!/usr/bin/env python
import rospy
import tf
from gazebo_msgs.msg import LinkStates

count = 0

def callback(data):
	
	"""global count
	count += 1
	if count > 1000:
		count = 0
		print(data.name)"""
	
	# Get index

	try:
		cafe_table_idx = data.name.index("cafe_table::link")
		brick_idx = data.name.index("brick::Brick")

		# Get cafe_table pose and brick pose w.r.t. gazebo world
		cafe_table_pose = data.pose[cafe_table_idx]
		brick_pose = data.pose[brick_idx]

		# Broadcast cafe table pose w.r.t. the gazebo world into tf tree
		br_cafe_table = tf.TransformBroadcaster()
		br_cafe_table.sendTransform((cafe_table_pose.position.x, cafe_table_pose.position.y, cafe_table_pose.position.z),
			(cafe_table_pose.orientation.x, cafe_table_pose.orientation.y, cafe_table_pose.orientation.z, cafe_table_pose.orientation.w),
			rospy.Time.now(), "cafe_table", "gazebo_world")

		# Broadcast brick pose w.r.t. the gazebo world into tf tree
		br_brick = tf.TransformBroadcaster()
		br_brick.sendTransform((brick_pose.position.x, brick_pose.position.y, brick_pose.position.z),
			(brick_pose.orientation.x, brick_pose.orientation.y, brick_pose.orientation.z, brick_pose.orientation.w),
			rospy.Time.now(), "brick", "gazebo_world")

	except ValueError:
		pass

def gazebo_link_subscriber():
	rospy.init_node('gazebo_link_subscriber')
	rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	gazebo_link_subscriber()
