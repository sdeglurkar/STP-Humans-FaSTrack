#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

sim_positions = {0:(12, 10), 1:(11, 10), 2:(10, 10), 3:(10, 11), 4:(9, 12), 5:(8, 13), \
    6:(7, 14), 7:(7, 15), 8:(7, 16), 9:(6, 15), 10:(5, 16), 11:(4, 17)}
real_positions = {0:(-0.38, -1.15), 1:(-0.765, -1.15), 2:(-1.15, -1.15), 3:(-1.15, -0.765), 4:(-1.535, -0.38), \
    5:(-1.92, 0.005), 6:(-2.305, 0.39), 7:(-2.305, 0.775), 8:(-2.305, 1.16), 9:(-2.69, 0.775), 10:(-3.075, 1.16), \
    11:(-3.46, 1.545)}


class Human:

    def __init__(self, my_positions = real_positions):
        rospy.init_node('human_pose_giver', anonymous=True)
        self.my_positions = my_positions
        self.pub = rospy.Publisher('human_pose', Point, queue_size=10)
        self.marker_pub = rospy.Publisher('human_pose_marker', Marker, queue_size=10)
        
        
    def interpolate(self, time):
        times = self.my_positions.keys()
        if time in times:
            return self.my_positions[time]
        elif time > times[-1]:
            last_time = times[-1]
            return self.my_positions[last_time]
        else:
            for i in range(len(times) - 1):
                if times[i] < time and times[i + 1] > time:
                    prev_t = times[i]
                    next_t = times[i + 1]
                    prev = self.my_positions[prev_t]
                    next = self.my_positions[next_t]
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
            self.marker_pub.publish(self.pose_to_marker(pose, 1, 0, 0, 1))
            rate.sleep()


    def pose_to_marker(self, pose, r, g, b, alpha):
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
        marker_pose.color.r = r
        marker_pose.color.g = g
        marker_pose.color.b = b
        marker_pose.color.a = alpha

        return marker_pose


if __name__ == '__main__':
    try:
        h = Human()
        h.give_human_pose()
    except rospy.ROSInterruptException:
        pass


