#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point

class DummyDoer(object):
	def __init__(self):
		self.node = rospy.init_node('dummydoer', anonymous=True)
		self.sub = rospy.Subscriber('dummy_pose', Point, self.callback)
		self.pub = rospy.Publisher('response', Point, queue_size=10)

		#rate = rospy.Rate(10)
		#while not rospy.is_shutdown():
		#	rate.sleep()
		rospy.spin()

	def callback(self, msg):
		pose = Point()
		pose.x = msg.x
		pose.y = msg.y
		pose.z = msg.z
		rospy.loginfo(pose)
		self.pub.publish(pose)


if __name__ == '__main__':
    try:
    	d = DummyDoer()
    except rospy.ROSInterruptException:
        pass
