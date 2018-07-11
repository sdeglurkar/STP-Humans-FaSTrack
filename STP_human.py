#!/usr/bin/env python

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
import pedestrian_prediction.pp.mdp.expanded as ex 
import pedestrian_prediction.pp.inference.hardmax.state as st 


class STP_Human:
	def __init__(self):
		self.load_parameters()
		rospy.init_node('human_pose_listener', anonymous=True)
		self.time_counter = rospy.get_time()
		self.sub = rospy.Subscriber('human_pose', Point, self.human_callback)
		self.pub0 = rospy.Publisher('robot_pose0', Point, queue_size=10)
		self.pub1 = rospy.Publisher('robot_pose1', Point, queue_size=10)
		self.pub2 = rospy.Publisher('robot_pose2', Point, queue_size=10)
		self.marker_pub0 = rospy.Publisher('robot_pose_marker0', Marker, queue_size=10)
		self.marker_pub1 = rospy.Publisher('robot_pose_marker1', Marker, queue_size=10)
		self.marker_pub2 = rospy.Publisher('robot_pose_marker2', Marker, queue_size=10)
		self.static_marker_pub = rospy.Publisher('static_obs_marker', Marker, queue_size=10)

		self.debugger_pub = rospy.Publisher('stp_debugger', String, queue_size=10)

		self.static_obs = {}
		self.static_obs[0] = []
		self.static_obs_poses = []
		for i in range(self.sim_length):
			row = [False for j in range(self.sim_width)]
			self.static_obs[0].append(row)
		for i in range(5):
			for j in range(3):
				self.static_obs[0][i + 12][j + 14] = True
				sim_pose = Point()
				sim_pose.x = i + 12
				sim_pose.y = j + 14
				sim_pose.z = 0.0
				(sim_pose.x, sim_pose.y) = self.sim_to_real_coord((sim_pose.x, sim_pose.y))
				self.static_obs_poses.append(sim_pose) #Now it's the real pose though

		rospy.spin()


	def load_parameters(self):
		self.human_goals = [(4, 17)] 
		self.num_robots = 3
		self.robot_starts = [(5, 5), (5, 20), (20, 15)]
		self.robot_goals = [[23, 23], [8, 2], [17, 4]] 

		# Sim grid is only positive (x, y)
		self.sim_length = 26 #x-direction
		self.sim_width = 26 #y-direction
		# Real grid dimensions
		self.real_length = 10
		self.real_width = 10
		self.real_upper = (5, 5)
		self.real_lower = (-5, -5)

		self.gridworld = ex.GridWorldExpanded(self.sim_length, self.sim_width)
		self.dest_list = [self.gridworld.coor_to_state(g[0], g[1]) for g in self.human_goals]
		self.betas = [0.2, 0.5, 1, 1.5, 3, 5, 8]
		self.fwd_tsteps = 7
		self.collision_threshold = 0.01
		self.tracking_error_bounds = [0, 0, 0] #tracking_error_bounds[i] is the teb planner i plans with

		self.possible_planners = [astar_STP_human.AStarPlanner(self.robot_starts[0], self.sim_length, self.sim_width, self.tracking_error_bounds[0]), \
							astar_STP_human.AStarPlanner(self.robot_starts[1], self.sim_length, self.sim_width, self.tracking_error_bounds[1]), \
							astar_STP_human.AStarPlanner(self.robot_starts[2], self.sim_length, self.sim_width, self.tracking_error_bounds[2])]
		self.planners = []
		for i in range(self.num_robots):
			self.planners.append(self.possible_planners[i])

		self.deltat = 1
		self.global_time = 0

		self.obstacle_traj = []



	def robot_follow_plan(self, planner, traj, dt):
		for point in traj:
			if point[2] < dt:
				planner.curr_pos = (point[0], point[1])
			else:
				break


	def traj_to_obmap(self, robot_traj, robot_size, tracking_error_bound): #obmap contains True at (x, y) if there is an obstacle at (x, y)
		obmap = {}
		for k in range(len(robot_traj)):
			time_stamp = robot_traj[k][2]
			obmap[time_stamp] = []
			for i in range(self.sim_length):
				row = [False for j in range(self.sim_width)]
				obmap[time_stamp].append(row)
			x = robot_traj[k][0]
			y = robot_traj[k][1]
			radius = math.ceil(robot_size + tracking_error_bound)
			rectangular_radius = math.sqrt(radius**2 + radius**2) #rectangle that circumscribes the circle defined by teb 
			for row in range(len(obmap[time_stamp])):
				for col in range(len(obmap[time_stamp][0])):
					dist_to_node = math.sqrt((row - x)**2 + (col - y)**2)
					if dist_to_node <= rectangular_radius:
						obmap[time_stamp][row][col] = True

		return obmap


	def sim_to_real_coord(self, sim_coord):
		return (sim_coord[0] * (float(self.real_length)/self.sim_length) + self.real_lower[0], \
			sim_coord[1] * (float(self.real_width)/self.sim_width) + self.real_lower[1])

	def real_to_sim_coord(self, real_coord):
		return ((real_coord[0] - self.real_lower[0]) * (float(self.sim_length)/self.real_length), \
			(real_coord[1] - self.real_lower[1]) * (float(self.sim_width)/self.real_width))


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
		marker_pose.scale.x = 0.5
		marker_pose.scale.y = 0.5
		marker_pose.scale.z = 0.5
		marker_pose.color.r = r
		marker_pose.color.g = g
		marker_pose.color.b = b
		marker_pose.color.a = alpha

		return marker_pose


	def poses_to_marker(self, r, g, b, alpha):
		marker = Marker()
		marker.header.frame_id = "world"
		marker.header.stamp = rospy.Time().now()
		marker.type = marker.SPHERE_LIST
		marker.action = marker.ADD
		marker.pose.orientation.w = 1
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
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
			grid_msg.header.stamp = curr_time + rospy.Duration(t*self.deltat)
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
		self.occupancy_grids = [None]*self.fwd_tsteps

		for i, grid in enumerate(msg.gridarray):
			self.occupancy_grids[i] = grid.data



	def human_callback(self, msg):
		x = msg.x
		y = msg.y
		(x, y) = self.real_to_sim_coord((x, y))
		pose0 = Point()
		pose1 = Point()
		pose2 = Point()
		t = rospy.get_time() - self.time_counter

		self.static_marker_pub.publish(self.poses_to_marker(1, 1, 1, 1))

		if self.num_robots >= 1:
			pose0.x = self.planners[0].curr_pos[0]
			pose0.y = self.planners[0].curr_pos[1]
			pose0.z = 0.0
			rospy.loginfo(pose0)
			self.pub0.publish(pose0)
			self.marker_pub0.publish(self.pose_to_marker(pose0, 0, 0, 1, 1))
		if self.num_robots >= 2:
			pose1.x = self.planners[1].curr_pos[0]
			pose1.y = self.planners[1].curr_pos[1]
			pose1.z = 0.0
			rospy.loginfo(pose1)
			self.pub1.publish(pose1)
			self.marker_pub1.publish(self.pose_to_marker(pose1, 0, 1, 0, 1))
		if self.num_robots >= 3:
			pose2.x = self.planners[2].curr_pos[0]
			pose2.y = self.planners[2].curr_pos[1]
			pose2.z = 0.0
			rospy.loginfo(pose2)
			self.pub2.publish(pose2)
			self.marker_pub2.publish(self.pose_to_marker(pose2, 1, 1, 1, 1))
    	

		if math.fabs(t - self.deltat) < 0.1:	
			self.global_time += t
			self.obstacle_traj.append((x, y))
			rospy.loginfo("Human is at (" + str(x) + "," + str(y) + ") at time " + str(self.global_time))
			
			for i in range(len(self.planners)):
				rospy.loginfo("Robot " + str(i) + " is at (" + str(self.planners[i].curr_pos[0]) + \
						"," + str(self.planners[i].curr_pos[1]) + ") at time " + str(self.global_time))

	
			(occupancy_grids, beta_occu, dest_beta_prob) = st.infer_joint(self.gridworld, 
					self.dest_list, self.betas, T=self.fwd_tsteps, use_gridless=True, traj=self.obstacle_traj, verbose_return=True)
			occupancy_grids_2D = [np.reshape(grid, (self.sim_length, self.sim_width)) for grid in occupancy_grids] 


			obmaps = [occupancy_grids_2D, self.static_obs]
			robot_trajs = []
			robot0_traj = self.planners[0].plan_traj(self.robot_goals[0], obmaps, self.collision_threshold)
			
			#traj_str = ""
			#for point in robot0_traj:
			#	traj_str += str(point[0])
			#	traj_str += ","
			#	traj_str += str(point[1])
			#	traj_str += ","
			#	traj_str += str(point[2])
			#	traj_str += " "
			#self.debugger_pub.publish(traj_str)
			
			robot_trajs.append(robot0_traj)
			obmaps.append(self.traj_to_obmap(robot0_traj, self.planners[0].robot_size, self.tracking_error_bounds[0]))

			for i in range(len(self.planners) - 1):
				robot_traj = self.planners[i + 1].plan_traj(self.robot_goals[i + 1], obmaps, self.collision_threshold)
				robot_trajs.append(robot_traj)
				obmaps.append(self.traj_to_obmap(robot_traj, self.planners[i + 1].robot_size, self.tracking_error_bounds[i + 1]))

			for i in range(len(self.planners)):
				self.robot_follow_plan(self.planners[i], robot_trajs[i], self.deltat)
		

			self.time_counter = rospy.get_time()


			
		

if __name__ == '__main__':
    try:
    	STPHuman = STP_Human()
    except rospy.ROSInterruptException:
        pass

