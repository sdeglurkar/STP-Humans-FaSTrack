#!/usr/bin/env python

# TODO: DELETE STP'S PEDESTRIAN PREDICTION #

import numpy as np
import time
import math
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import String
from crazyflie_human.msg import OccupancyGridTime, ProbabilityGrid

import astar_STP_human 


class STP_Human:
	def __init__(self):
		self.load_parameters()
		rospy.init_node('STP', anonymous=True)
		self.time_counter = rospy.get_time()
		self.occugrid_sub = rospy.Subscriber('occupancy_grid_time', OccupancyGridTime, self.occugrid_callback)
		self.pub0 = rospy.Publisher('robot_pose0', Point, queue_size=10)
		self.pub1 = rospy.Publisher('robot_pose1', Point, queue_size=10)
		self.pub2 = rospy.Publisher('robot_pose2', Point, queue_size=10)
		self.marker_pub0 = rospy.Publisher('robot_pose_marker0', Marker, queue_size=10)
		self.marker_pub1 = rospy.Publisher('robot_pose_marker1', Marker, queue_size=10)
		self.marker_pub2 = rospy.Publisher('robot_pose_marker2', Marker, queue_size=10)
		self.static_marker_pub = rospy.Publisher('static_obs_marker', Marker, queue_size=10)

		self.debugger_pub = rospy.Publisher('stp_debugger', String, queue_size=10)

		self.static_obs = {}
		self.static_obs[0] = np.array(np.zeros((self.sim_length, self.sim_width, self.sim_height)), dtype = bool)
		self.static_obs_poses = [] #for visualization
		for i in range(5):
			for j in range(3):
				for k in range(2):
					self.static_obs[0][i + 8][j + 10][k] = True
					sim_pose = Point()
					sim_pose.x = i + 8
					sim_pose.y = j + 10
					sim_pose.z = k
					(sim_pose.x, sim_pose.y) = self.sim_to_real_coord((sim_pose.x, sim_pose.y))
					self.static_obs_poses.append(sim_pose) #Now it's the real pose though
		goal0 = Point()
		goal0.x = self.robot_goals[0][0]
		goal0.y = self.robot_goals[0][1]
		goal0.z = self.robot_goals[0][2]
		(goal0.x, goal0.y) = self.sim_to_real_coord((goal0.x, goal0.y))
		self.static_obs_poses.append(goal0)
		goal1 = Point()
		goal1.x = self.robot_goals[1][0]
		goal1.y = self.robot_goals[1][1]
		goal1.z = self.robot_goals[1][2]
		(goal1.x, goal1.y) = self.sim_to_real_coord((goal1.x, goal1.y))
		self.static_obs_poses.append(goal1)


		# DEBUGGING  - PUBLISH MARKERS FOR CORNERS OF REAL GRID #
		dummy = Point()
		dummy.x = 25
		dummy.y = 25
		dummy.z = 0
		(dummy.x, dummy.y) = self.sim_to_real_coord((dummy.x, dummy.y))
		self.static_obs_poses.append(dummy)
		dummy = Point()
		dummy.x = 0
		dummy.y = 0
		dummy.z = 0
		(dummy.x, dummy.y) = self.sim_to_real_coord((dummy.x, dummy.y))
		self.static_obs_poses.append(dummy)
		dummy = Point()
		dummy.x = 0
		dummy.y = 25
		dummy.z = 0
		(dummy.x, dummy.y) = self.sim_to_real_coord((dummy.x, dummy.y))
		self.static_obs_poses.append(dummy)
		dummy = Point()
		dummy.x = 25
		dummy.y = 0
		dummy.z = 0
		(dummy.x, dummy.y) = self.sim_to_real_coord((dummy.x, dummy.y))
		self.static_obs_poses.append(dummy)


		rospy.spin()


	def load_parameters(self):
		self.num_robots = rospy.get_param("stp/num_robots")
		self.robot_starts = rospy.get_param("stp/robot_starts")
		self.robot_goals = rospy.get_param("stp/robot_goals")
		
		# Real grid dimensions
		self.real_upper = rospy.get_param("state/upper")
		self.real_lower = rospy.get_param("state/lower")
		self.real_length = self.real_upper[0] - self.real_lower[0]
		self.real_width = self.real_upper[1] - self.real_lower[1]
		# Sim grid is only positive (x, y) 
		self.sim_length = int(rospy.get_param("pred/sim_width")) #x-direction
		self.sim_width = int(rospy.get_param("pred/sim_height")) #y-direction
		self.sim_height = int(self.real_upper[2])

		self.res = rospy.get_param("pred/resolution")
		self.fwd_tsteps = rospy.get_param("pred/fwd_tsteps")
		self.collision_threshold = rospy.get_param("pred/prob_thresh")
		self.tracking_error_bounds = rospy.get_param("stp/tracking_error_bounds")

		self.possible_planners = [astar_STP_human.AStarPlanner(self.robot_starts[0], self.sim_length, self.sim_width, self.sim_height, self.tracking_error_bounds[0]), \
							astar_STP_human.AStarPlanner(self.robot_starts[1], self.sim_length, self.sim_width, self.sim_height, self.tracking_error_bounds[1]), \
							astar_STP_human.AStarPlanner(self.robot_starts[2], self.sim_length, self.sim_width, self.sim_height, self.tracking_error_bounds[2])]
		self.planners = []
		for i in range(self.num_robots):
			self.planners.append(self.possible_planners[i])

		self.delta_t = rospy.get_param("stp/delta_t")
		self.global_time = 0



	def robot_follow_plan(self, planner, traj, dt):
		for point in traj:
			if point[3] < dt:
				planner.curr_pos = (point[0], point[1], point[2])
			else:
				break


	def traj_to_obmap(self, robot_traj, robot_size, tracking_error_bound): #obmap contains True at (x, y) if there is an obstacle at (x, y)
		obmap = {}
		for k in range(len(robot_traj)):
			time_stamp = robot_traj[k][3]
			obmap[time_stamp] = []
			obmap[time_stamp] = [[[False for z in range(self.sim_height)] \
				for y in range(self.sim_width)] for x in range(self.sim_length)]
			x = robot_traj[k][0]
			y = robot_traj[k][1]
			z = robot_traj[k][2]
			radius = math.ceil(robot_size + tracking_error_bound)
			cube_radius = math.sqrt(radius**2 + radius**2 + radius**2) #cube that circumscribes the sphere defined by teb 
			for i in range(len(obmap[time_stamp])):
				for j in range(len(obmap[time_stamp][0])):
					for k in range(len(obmap[time_stamp][0][0])):
						dist_to_node = math.sqrt((i - x)**2 + (j - y)**2 + (k - z)**2)
						if dist_to_node <= cube_radius:
							obmap[time_stamp][i][j][k] = True

		return obmap


	def sim_to_real_coord(self, sim_coord):
		return (sim_coord[0] * self.res + self.real_lower[0], \
			self.real_upper[1] - sim_coord[1] * self.res)


	def real_to_sim_coord(self, real_coord):
		return ((real_coord[0] - self.real_lower[0]) * (1/self.res), \
			(self.real_upper[1] - real_coord[1]) * (1/self.res))


	def pose_to_marker(self, pose, r, g, b, alpha):
		marker_pose = Marker()
		marker_pose.header.frame_id = "world"
		marker_pose.header.stamp = rospy.Time().now()
		marker_pose.type = marker_pose.SPHERE
		marker_pose.action = marker_pose.ADD
		marker_pose.pose.orientation.w = 1
		(x, y) = self.sim_to_real_coord((pose.x, pose.y)) 
		marker_pose.pose.position.x = x
		marker_pose.pose.position.y = y
		marker_pose.pose.position.z = pose.z
		marker_pose.scale.x = 2 * ((self.res/2) + (self.tracking_error_bounds[0] * self.res))
		marker_pose.scale.y = 2 * ((self.res/2) + (self.tracking_error_bounds[0] * self.res))
		marker_pose.scale.z = 2 * ((self.res/2) + (self.tracking_error_bounds[0] * self.res))
		marker_pose.color.r = r
		marker_pose.color.g = g
		marker_pose.color.b = b
		marker_pose.color.a = alpha

		return marker_pose


	def poses_to_marker(self, r, g, b, alpha): #For the static obstacle
		marker = Marker()
		marker.header.frame_id = "world"
		marker.header.stamp = rospy.Time().now()
		marker.type = marker.SPHERE_LIST
		marker.action = marker.ADD
		marker.pose.orientation.w = 1
		marker.scale.x = self.res
		marker.scale.y = self.res
		marker.scale.z = self.res
		marker.color.r = r
		marker.color.g = g
		marker.color.b = b
		marker.color.a = alpha
		marker.points = self.static_obs_poses

		return marker


	def grid_to_message(self): #Converts OccupancyGridTime structure to ROS msg
		timed_grid = OccupancyGridTime()
		timed_grid.gridarray = [None]*self.fwd_tsteps
		timed_grid.object_num = int(self.human_number) if self.human_number is not "" else 0

		curr_time = rospy.Time.now()

		for t in range(self.fwd_tsteps):
			grid_msg = ProbabilityGrid()

			# Set up the header.
			grid_msg.header.stamp = curr_time + rospy.Duration(t*self.delta_t)
			grid_msg.header.frame_id = "/world"

			# .info is a nav_msgs/MapMetaData message. 
			grid_msg.resolution = self.res
			grid_msg.width = self.sim_width
			grid_msg.height = self.sim_height

			# Rotated maps are not supported... 
			origin_x=0.0 
			origin_y=0.0 
			grid_msg.origin = Pose(Point(origin_x, origin_y, 0), Quaternion(0, 0, 0, 1))

			# convert to list of doubles from 0-1
			grid_msg.data = list(self.occupancy_grids[t])

			timed_grid.gridarray[t] = grid_msg

			return timed_grid


	def from_ROSMsg(self, msg):
		"""
		Converts occugrid message into structure
		"""
		occupancy_grids = {}
		tcount = 0
		#occupancy_grids = [None]*self.fwd_tsteps

		for i in range(len(msg.gridarray)):
			if i > 0:
				tcount += msg.gridarray[i].header.stamp.secs - msg.gridarray[i - 1].header.stamp.secs
			occupancy_grids[tcount] = msg.gridarray[i].data
		
		#for i, grid in enumerate(msg.gridarray):
			#occupancy_grids[i] = grid.data

		return occupancy_grids



	def occugrid_callback(self, msg):
		occupancy_grids = self.from_ROSMsg(msg)
		occupancy_grids_2D = {}
		for time, grid in occupancy_grids.items():
			occupancy_grids_2D[time] = np.reshape(grid, (self.sim_length, self.sim_width))
		#occupancy_grids_2D = [np.reshape(grid, (self.sim_length, self.sim_width)) for grid in occupancy_grids]

		pose0 = Point()
		pose1 = Point()
		pose2 = Point()
		t = rospy.get_time() - self.time_counter

		self.static_marker_pub.publish(self.poses_to_marker(0, 0, 0, 1))

		if self.num_robots >= 1:
			pose0.x = self.planners[0].curr_pos[0]
			pose0.y = self.planners[0].curr_pos[1]
			pose0.z = self.planners[0].curr_pos[2]
			rospy.loginfo(pose0)
			self.pub0.publish(pose0)
			self.marker_pub0.publish(self.pose_to_marker(pose0, 0, 0, 1, 1))
		if self.num_robots >= 2:
			pose1.x = self.planners[1].curr_pos[0]
			pose1.y = self.planners[1].curr_pos[1]
			pose1.z = self.planners[1].curr_pos[2]
			rospy.loginfo(pose1)
			self.pub1.publish(pose1)
			self.marker_pub1.publish(self.pose_to_marker(pose1, 0, 1, 0, 1))
		if self.num_robots >= 3:
			pose2.x = self.planners[2].curr_pos[0]
			pose2.y = self.planners[2].curr_pos[1]
			pose2.z = self.planners[2].curr_pos[2]
			rospy.loginfo(pose2)
			self.pub2.publish(pose2)
			self.marker_pub2.publish(self.pose_to_marker(pose2, 1, 1, 1, 1))
    	

		if math.fabs(t - self.delta_t) < 0.1:	
			self.global_time += t
			for i in range(len(self.planners)):
				rospy.loginfo("Robot " + str(i) + " is at (" + str(self.planners[i].curr_pos[0]) + \
						"," + str(self.planners[i].curr_pos[1]) + "," + str(self.planners[i].curr_pos[2]) + \
						") at time " + str(self.global_time))

			obmaps = [occupancy_grids_2D, self.static_obs]
			robot_trajs = []
			robot0_traj = self.planners[0].plan_traj(self.robot_goals[0], obmaps, self.collision_threshold)
			
			
			# DEBUGGING - PUBLISH ROBOT 0'S TRAJECTORY #
			traj_str = ""
			for point in robot0_traj:
				traj_str += str(point[0])
				traj_str += ","
				traj_str += str(point[1])
				traj_str += ","
				traj_str += str(point[2])
				traj_str += " "
				traj_str += str(point[3])
				traj_str += " "
			self.debugger_pub.publish(traj_str)
			
			robot_trajs.append(robot0_traj)
			obmaps.append(self.traj_to_obmap(robot0_traj, self.planners[0].robot_size, self.tracking_error_bounds[0]))

			for i in range(len(self.planners) - 1):
				robot_traj = self.planners[i + 1].plan_traj(self.robot_goals[i + 1], obmaps, self.collision_threshold)
				robot_trajs.append(robot_traj)
				obmaps.append(self.traj_to_obmap(robot_traj, self.planners[i + 1].robot_size, self.tracking_error_bounds[i + 1]))

			for i in range(len(self.planners)):
				self.robot_follow_plan(self.planners[i], robot_trajs[i], self.delta_t)
		

			self.time_counter = rospy.get_time()

			

if __name__ == '__main__':
    try:
    	STPHuman = STP_Human()
    except rospy.ROSInterruptException:
        pass

