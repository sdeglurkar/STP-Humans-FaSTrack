import numpy as np
import time
import math
import matplotlib.pyplot as plt
from IPython import display

import astar_STP_human_3D
import pedestrian_prediction.pp.mdp.expanded as ex
import pedestrian_prediction.pp.inference.hardmax.state as st

human_goals = [(4, 17, 0)]
num_robots = 3
robot_starts = [(5, 5, 0), (5, 10, 0), (10, 23, 0)]
robot_goals = [[23, 23, 0], [20, 14, 0], [15, 5, 0]]
grid_length = 26  # x-direction
grid_width = 26  # y-direction
grid_height = 26 # z-direction
gridworld = ex.GridWorldExpanded(grid_length, grid_width)
dest_list = [gridworld.coor_to_state(g[0], g[1]) for g in human_goals]
betas = [0.2, 0.5, 1, 1.5, 3, 5, 8]
fwd_tsteps = 7
collision_threshold = 0.01
show_animation = True
show_prob_dist = False
show_teb = True
tracking_error_bounds = [1, 1, 0]

possible_planners = [astar_STP_human_3D.AStarPlanner(robot_starts[0], grid_length, grid_width, grid_height, tracking_error_bounds[0]), \
                     astar_STP_human_3D.AStarPlanner(robot_starts[1], grid_length, grid_width, grid_height, tracking_error_bounds[1]), \
                     astar_STP_human_3D.AStarPlanner(robot_starts[2], grid_length, grid_width, grid_height, tracking_error_bounds[2])]
planners = []
for i in range(num_robots):
    planners.append(possible_planners[i])

planner_colors_pos = ['bo', 'go', 'ro', 'yo']
planner_colors_goals = ['bx', 'gx', 'rx', 'yx']


def get_human_position(t):
    if t == 0:
        return (12, 10, 0)
    elif t == 1:
        return (11, 10, 0)
    elif t == 2:
        return (10, 10, 0)
    elif t == 3:
        return (10, 11, 0)
    elif t == 4:
        return (9, 12, 0)
    elif t == 5:
        return (8, 13, 0)
    elif t == 6:
        return (7, 14, 0)
    elif t == 7:
        return (7, 15, 0)
    elif t == 8:
        return (7, 16, 0)
    elif t == 9:
        return (6, 15, 0)
    elif t == 10:
        return (5, 16, 0)
    elif t == 11:
        return (4, 17, 0)


def robot_follow_plan(planner, traj, dt):
    for point in traj:
        if point[3] < dt:
            planner.curr_pos = (point[0], point[1], point[2])
        else:
            break


def traj_to_obmap(robot_traj, robot_size,
                  tracking_error_bound):  # obmap contains True at (x, y, z) if there is an obstacle at (x, y, z)
    obmap = {}
    for r in range(len(robot_traj)):
        time_stamp = robot_traj[k][3]
        obmap[time_stamp] = [[[False for x in range(grid_length)]for y in range(grid_width)] for z in range(grid_height)]
        x = robot_traj[r][0]
        y = robot_traj[r][1]
        z = robot_traj[r][2]
        radius = math.ceil(robot_size + tracking_error_bound)
        cube_radius = math.sqrt(
            radius ** 2 + radius ** 2 + radius ** 2)  # rectangle that circumscribes the circle defined by teb
        for i in range(len(obmap[time_stamp])):
            for j in range(len(obmap[time_stamp][0])):
                for k in range(len(obmap[time_stamp][0][0])):
                    dist_to_node = math.sqrt((i - x) ** 2 + (j - y) ** 2 + (k - z) ** 2)
                    if dist_to_node <= cube_radius:
                        obmap[time_stamp][i][j][k] = True
    return obmap


T = 20
dt = 1
t = 0
obstacle_traj = []

static_obs = {}
static_obs[0] = [[[False for x in range(grid_length)]for y in range(grid_width)] for z in range(grid_height)]
for i in range(5):
    for j in range(3):
        for k in range(3):
            static_obs[0][i + 12][j + 14][k] = True

while t < T:
    if get_human_position(t) is not None:
        (x, y, z) = get_human_position(t)
        obstacle_traj.append((x, y, z))
        print("Human is at ", (x, y, z), " at time ", t)
        if show_animation:
            for i in range(num_robots):
                plt.plot(robot_goals[i][0], robot_goals[i][1], planner_colors_goals[i])
            for goal in human_goals:
                plt.plot(goal[0], goal[1], 'kx')

            for row in range(len(static_obs[0])):
                for col in range(len(static_obs[0][0])):
                    if static_obs[0][row][col]:
                        plt.plot(row, col, 'ko')
            plt.plot(0, 0)
            plt.plot(grid_length, grid_width)
            plt.plot(x, y, 'ko')
    else:
        print("Human is at (4,17) at time ", t)
        if show_animation:
            for i in range(num_robots):
                plt.plot(robot_goals[i][0], robot_goals[i][1], planner_colors_goals[i])
            for goal in human_goals:
                plt.plot(goal[0], goal[1], 'kx')

            for row in range(len(static_obs[0])):
                for col in range(len(static_obs[0][0])):
                    if static_obs[0][row][col]:
                        plt.plot(row, col, 'ko')
            plt.plot(0, 0)
            plt.plot(grid_length, grid_width)
            plt.plot(4, 17, 'ko')

    for i in range(len(planners)):
        print("Robot ", i, " is at ", planners[i].curr_pos, " at time ", t)

    if show_animation:
        for i in range(len(planners)):
            plt.plot(planners[i].curr_pos[0], planners[i].curr_pos[1], planner_colors_pos[i])
            if show_teb:
                circle = plt.Circle((planners[i].curr_pos[0], planners[i].curr_pos[1]), \
                                    math.ceil(tracking_error_bounds[i] + 0.5), color=planner_colors_pos[i][0],
                                    fill=False)
                fig = plt.gcf()
                ax = fig.gca()
                ax.add_artist(circle)
        plt.pause(1)

    (occupancy_grids, beta_occu, dest_beta_prob) = st.infer_joint(gridworld,
                                                                  dest_list, betas, T=fwd_tsteps, use_gridless=True,
                                                                  traj=obstacle_traj, verbose_return=True)
    occupancy_grids_2D = [np.reshape(grid, (grid_length, grid_width)) for grid in occupancy_grids]

    if show_animation and show_prob_dist:
        obstacle_grid = occupancy_grids_2D[2]

        for row in range(len(obstacle_grid)):
            for col in range(len(obstacle_grid[0])):
                if obstacle_grid[row][col] >= 0.005 and obstacle_grid[row][col] < 0.01:
                    plt.plot(row, col, 'bo', alpha=0.2)
                elif obstacle_grid[row][col] >= 0.01 and obstacle_grid[row][col] < 0.015:
                    plt.plot(row, col, 'go', alpha=0.2)
                elif obstacle_grid[row][col] >= 0.015 and obstacle_grid[row][col] < 0.02:
                    plt.plot(row, col, 'yo', alpha=0.2)
                elif obstacle_grid[row][col] > 0.02:
                    plt.plot(row, col, 'ro', alpha=0.2)

    obmaps = [occupancy_grids_2D, static_obs]
    robot_trajs = []
    robot0_traj = planners[0].plan_traj(robot_goals[0], obmaps, collision_threshold)
    robot_trajs.append(robot0_traj)
    obmaps.append(traj_to_obmap(robot0_traj, planners[0].robot_size, tracking_error_bounds[0]))

    for i in range(len(planners) - 1):
        robot_traj = planners[i + 1].plan_traj(robot_goals[i + 1], obmaps, collision_threshold)
        robot_trajs.append(robot_traj)
        obmaps.append(traj_to_obmap(robot_traj, planners[i + 1].robot_size, tracking_error_bounds[i + 1]))

    for i in range(len(planners)):
        robot_follow_plan(planners[i], robot_trajs[i], dt)

    t += dt

if show_animation:
    plt.show()
