#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class Dummy(object):
	def __init__(self):
		rospy.init_node('dummy', anonymous=True)
		self.pub = rospy.Publisher('dummy_pose', Point, queue_size=10)
		self.marker_pub = rospy.Publisher('dummy_pose_marker', Marker, queue_size=10)
		rate = rospy.Rate(10) # 10hz
		pose = Point()	

		while not rospy.is_shutdown():
			pose.x = 5.0
			pose.y = 5.0
			pose.z = 0.0
			rospy.loginfo(pose)
			self.pub.publish(pose)
			self.marker_pub.publish(self.pose_to_marker(pose))
			rate.sleep()


	def pose_to_marker(self, pose):
		marker_pose = Marker()
		marker_pose.header.frame_id = "world"
		marker_pose.header.stamp = rospy.Time().now()
		marker_pose.type = marker_pose.SPHERE
		marker_pose.action = marker_pose.ADD
		marker_pose.pose.orientation.w = 1
		marker_pose.pose.position.x = pose.x
		marker_pose.pose.position.y = pose.y
		marker_pose.pose.position.z = pose.z
		marker_pose.scale.x = 1
		marker_pose.scale.y = 1
		marker_pose.scale.z = 1
		marker_pose.color.r = 1
		marker_pose.color.g = 0
		marker_pose.color.b = 0
		marker_pose.color.a = 1

		return marker_pose
		

if __name__ == '__main__':
    try:
    	d = Dummy()
    except rospy.ROSInterruptException:
        pass