#!/usr/bin/env python

import numpy as np
import time
import math
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import Point

import astar_STP_human
import pedestrian_prediction.pp.mdp.expanded as ex 
import pedestrian_prediction.pp.inference.hardmax.state as st 


class STP_Human:
	def __init__(self):
		self.load_parameters()

	def load_parameters(self):
		self.human_goals = [(4, 17)] 
		self.num_robots = 1
		self.robot_starts = [(5, 5), (5, 10), (10, 23)]
		self.robot_goals = [[23, 23], [20, 14], [15, 5]] 
		self.grid_length = 26 #x-direction
		self.grid_width = 26 #y-direction
		self.gridworld = ex.GridWorldExpanded(self.grid_length, self.grid_width)
		self.dest_list = [self.gridworld.coor_to_state(g[0], g[1]) for g in self.human_goals]
		self.betas = [0.2, 0.5, 1, 1.5, 3, 5, 8]
		self.fwd_tsteps = 7
		self.collision_threshold = 0.01
		self.show_animation = False
		self.show_prob_dist = False
		self.show_teb = False
		self.tracking_error_bounds = [0, 0, 0] #tracking_error_bounds[i] is the teb planner i plans with

		self.possible_planners = [astar_STP_human.AStarPlanner(self.robot_starts[0], self.grid_length, self.grid_width, self.tracking_error_bounds[0]), \
							astar_STP_human.AStarPlanner(self.robot_starts[1], self.grid_length, self.grid_width, self.tracking_error_bounds[1]), \
							astar_STP_human.AStarPlanner(self.robot_starts[2], self.grid_length, self.grid_width, self.tracking_error_bounds[2])]
		self.planners = []
		for i in range(self.num_robots):
			self.planners.append(self.possible_planners[i])

		self.planner_colors_pos = ['bo', 'go', 'ro', 'yo'] 
		self.planner_colors_goals = ['bx', 'gx', 'rx', 'yx']

		self.deltat = 1
		self.global_time = 0

		self.obstacle_traj = []

		self.static_obs = {}
		self.static_obs[0] = []
		for i in range(self.grid_length):
			row = [False for j in range(self.grid_width)]
			self.static_obs[0].append(row)
		for i in range(5):
			for j in range(3):
				self.static_obs[0][i + 12][j + 14] = True



	def get_human_pose(self): 
		rospy.init_node('human_pose_listener', anonymous=True)
		self.time_counter = rospy.get_time()
		rospy.Subscriber('human_pose', Point, self.human_callback)
		rospy.spin()


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
			for i in range(self.grid_length):
				row = [False for j in range(self.grid_width)]
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



	#T = 20
	#dt = 1
	#t = 0

	def human_callback(self, msg):
		x = msg.x
		y = msg.y
		t = rospy.get_time() - self.time_counter
		self.global_time += t
		
		#pub = rospy.Publisher('robot_pose', Point, queue_size=10)
    	#rospy.init_node('robot_pose_giver', anonymous=True)
    	#rate = rospy.Rate(10) # 10hz
    	#while not rospy.is_shutdown():
    	#	pose = Point()
    	#	pose.x, pose.y = plan_point[0], plan_point[1]
    	#	pose.z = 0.0
    	#	rospy.loginfo(pose)
    	#	pub.publish(pose)
    	#	rate.sleep()
    			
		if math.fabs(t - self.deltat) < 0.1:	
			self.obstacle_traj.append((x, y))
			rospy.loginfo("Human is at (" + str(x) + "," + str(y) + ") at time " + str(self.global_time))
			#print("Human is at ", (x,y), " at time ", self.global_time)
			if self.show_animation:
				for i in range(self.num_robots):
					plt.plot(self.robot_goals[i][0], self.robot_goals[i][1], self.planner_colors_goals[i])
				for goal in self.human_goals:
					plt.plot(goal[0], goal[1], 'kx')
			
				for row in range(len(self.static_obs[0])):
					for col in range(len(self.static_obs[0][0])):
						if self.static_obs[0][row][col]:
							plt.plot(row, col, 'ko')
				plt.plot(0, 0)
				plt.plot(self.grid_length, self.grid_width)
				plt.plot(x, y, 'ko')
		
	
			for i in range(len(self.planners)):
				rospy.loginfo("Robot " + str(i) + " is at (" + str(self.planners[i].curr_pos[0]) + "," + str(self.planners[i].curr_pos[1]) + ") at time " + str(self.global_time))
 				#print("Robot ", i, " is at ", self.planners[i].curr_pos, " at time ", self.global_time)
	
			if self.show_animation:
				for i in range(len(self.planners)):
					plt.plot(self.planners[i].curr_pos[0], self.planners[i].curr_pos[1], self.planner_colors_pos[i])
					if self.show_teb:
						circle = plt.Circle((self.planners[i].curr_pos[0], self.planners[i].curr_pos[1]), \
							math.ceil(self.tracking_error_bounds[i] + self.planners[i].robot_size), color = self.planner_colors_pos[i][0], fill = False)
						fig = plt.gcf()
						ax = fig.gca()
						ax.add_artist(circle)
				#plt.pause(1)
	

			(occupancy_grids, beta_occu, dest_beta_prob) = st.infer_joint(self.gridworld, 
					self.dest_list, self.betas, T=self.fwd_tsteps, use_gridless=True, traj=self.obstacle_traj, verbose_return=True)
			occupancy_grids_2D = [np.reshape(grid, (self.grid_length, self.grid_width)) for grid in occupancy_grids] 


			if self.show_animation and self.show_prob_dist:
				obstacle_grid = occupancy_grids_2D[2]

				for row in range(len(obstacle_grid)):
					for col in range(len(obstacle_grid[0])):
						if obstacle_grid[row][col] >= 0.005 and obstacle_grid[row][col] < 0.01:
							plt.plot(row, col, 'bo', alpha = 0.2)
						elif obstacle_grid[row][col] >= 0.01 and obstacle_grid[row][col] < 0.015:
							plt.plot(row, col, 'go', alpha = 0.2)
						elif obstacle_grid[row][col] >= 0.015 and obstacle_grid[row][col] < 0.02:
							plt.plot(row, col, 'yo', alpha = 0.2)
						elif obstacle_grid[row][col] > 0.02:
							plt.plot(row, col, 'ro', alpha = 0.2)


			obmaps = [occupancy_grids_2D, self.static_obs]
			robot_trajs = []
			robot0_traj = self.planners[0].plan_traj(self.robot_goals[0], obmaps, self.collision_threshold)
			robot_trajs.append(robot0_traj)
			obmaps.append(self.traj_to_obmap(robot0_traj, self.planners[0].robot_size, self.tracking_error_bounds[0]))

			for i in range(len(self.planners) - 1):
				robot_traj = self.planners[i + 1].plan_traj(self.robot_goals[i + 1], obmaps, self.collision_threshold)
				robot_trajs.append(robot_traj)
				obmaps.append(self.traj_to_obmap(robot_traj, self.planners[i + 1].robot_size, self.tracking_error_bounds[i + 1]))

			for i in range(len(self.planners)):
				self.robot_follow_plan(self.planners[i], robot_trajs[i], self.deltat)
				
			#for i in range(len(self.planners)):
				#self.give_plans(self.planners[i].curr_pos)
				#self.give_plans([5, 5])


			#t += dt
			self.time_counter = rospy.get_time()
		
		
		if self.show_animation:
			plt.show()	
			
			
			
	#def give_plans(self, plan_point):
	#	pub = rospy.Publisher('robot_pose', Point, queue_size=10)
    #	rospy.init_node('robot_pose_giver', anonymous=True)
    #	rate = rospy.Rate(10) # 10hz
    #	while not rospy.is_shutdown():
    #		pose = Point()
    #		pose.x, pose.y = plan_point[0], plan_point[1]
    #		pose.z = 0.0
    #		rospy.loginfo(pose)
    #		pub.publish(pose)
    #		rate.sleep()



if __name__ == '__main__':
    try:
    	STPHuman = STP_Human()
    	#STPHuman.give_plans([5, 5])
    	STPHuman.get_human_pose()
    except rospy.ROSInterruptException:
        pass


