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
		self.pose = Point()
		self.pose.x = 5.0
		self.pose.y = 5.0
		self.pose.z = 0.0
		
		self.marker_pose = Marker()
		self.marker_pose.header.frame_id = "world"
		self.marker_pose.header.stamp = rospy.Time().now()
		self.marker_pose.type = self.marker_pose.SPHERE
		self.marker_pose.action = self.marker_pose.ADD
		self.marker_pose.pose.orientation.w = 1
		self.marker_pose.pose.position.x = self.pose.x
		self.marker_pose.pose.position.y = self.pose.y
		self.marker_pose.pose.position.z = self.pose.z
		self.marker_pose.scale.x = 1
		self.marker_pose.scale.y = 1
		self.marker_pose.scale.z = 1
		self.marker_pose.color.r = 1
		self.marker_pose.color.g = 0
		self.marker_pose.color.b = 0
		self.marker_pose.color.a = 1



		while not rospy.is_shutdown():
			rospy.loginfo(self.pose)
			self.pub.publish(self.pose)
			self.marker_pub.publish(self.marker_pose)
    		rate.sleep()
	
	#def give_dummy_pose(self):
		


if __name__ == '__main__':
    try:
    	d = Dummy()
    	#d.give_dummy_pose()
    except rospy.ROSInterruptException:
        pass