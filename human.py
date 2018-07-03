#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

positions = {0:(12, 10), 1:(11, 10), 2:(10, 10), 3:(10, 11), 4:(9, 12), 5:(8, 13), \
    6:(7, 14), 7:(7, 15), 8:(7, 16), 9:(6, 15), 10:(5, 16), 11:(4, 17)}

class Human:

    def __init__(self, my_positions = positions):
        rospy.init_node('human_pose_giver', anonymous=True)
        self.my_positions = my_positions
        self.pub = rospy.Publisher('human_pose', Point, queue_size=10)
        self.marker_pub = rospy.Publisher('human_pose_marker', Marker, queue_size=10)
        
        
    def interpolate(self, time):
        times = positions.keys()
        if time in times:
            return positions[time]
        elif time > times[-1]:
            last_time = times[-1]
            return positions[last_time]
        else:
            for i in range(len(times) - 1):
                if times[i] < time and times[i + 1] > time:
                    prev_t = times[i]
                    next_t = times[i + 1]
                    prev = positions[prev_t]
                    next = positions[next_t]
                    x = prev[0] + (next[0] - prev[0]) * ((time - prev_t) / (next_t - prev_t)) 
                    y = prev[1] + (next[1] - prev[1]) * ((time - prev_t) / (next_t - prev_t))
                    return (x, y)


    def give_human_pose(self):
        rate = rospy.Rate(10) # 10hz
        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            curr_t = rospy.get_time() - start_time
            pose = Point()
            pose.x, pose.y = self.interpolate(curr_t)
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
        h = Human()
        h.give_human_pose()
    except rospy.ROSInterruptException:
        pass


