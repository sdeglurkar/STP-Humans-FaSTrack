#!/usr/bin/env python

import matplotlib.pyplot as plt
import math
import numpy as np

import rospy
from std_msgs.msg import String

class Node:

    def __init__(self, x, y, z, cost, pind, tstamp):
        self.x = x
        self.y = y
        self.z = z
        self.cost = cost
        self.pind = pind
        self.tstamp = tstamp

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.z) + "," + \
            str(self.cost) + "," + str(self.pind) + "," + str(self.tstamp)


class AStarPlanner:
    
    def __init__(self, start, grid_length, grid_width, grid_height, tracking_error_bound, \
                        grid_reso = 1, robot_size = 0.5, speed=3):
        self.start = start
        self.grid_length = grid_length #x direction
        self.grid_width = grid_width #y direction
        self.grid_height = grid_height
        self.tracking_error_bound = tracking_error_bound

        self.grid_reso = 1
        self.robot_size = robot_size
        self.speed = speed
        self.curr_pos = start

        self.debugger_pub = rospy.Publisher('planner_debugger', String, queue_size=10)


    def plan_traj(self, goal, obmaps, collision_threshold): #obmaps contains one 3D array and num_robots amount of dictionaries with 2D arrays as values
        self.debugger_pub.publish("Planning")
        print("Planning")
        nstart = Node(round(self.curr_pos[0]/self.grid_reso), round(self.curr_pos[1]/self.grid_reso), \
            round(self.curr_pos[2]/self.grid_reso), 0.0, -1, 0.0)
        #motion = self.get_motion_model()

        openset, closedset = dict(), dict()
        openset[self.calc_index(nstart)] = nstart

        gx = goal[0]
        gy = goal[1]
        gz = goal[2]

        start = rospy.Time().now().secs
        while 1:
            #for num in openset:
            #    print(openset[num].x, openset[num].y)
            c_id = min(
                openset, key=lambda o: openset[o].cost + self.calc_h(gx, gy, gz, openset[o].x, openset[o].y, openset[o].z))
            current = openset[c_id]
            print(current)

            if current.x == gx and current.y == gy and current.z == gz:
                print("Found goal")
                ngoal = Node(round(gx/self.grid_reso), round(gy/self.grid_reso), round(gz/self.grid_reso), \
                    current.cost, current.pind, current.tstamp)
                break

            # Remove the item from the open set
            del openset[c_id]
            # Add it to the closed set
            closedset[c_id] = current

            # expand search grid based on motion model
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    for k in [-1, 0, 1]:
                        node = Node(current.x + i, current.y + j, current.z + k, \
                            current.cost + self.calculate_motion_cost(i, j, k), c_id, \
                            current.tstamp + 1.0/self.speed)
                        n_id = self.calc_index(node)

                        if not self.verify_node(node, obmaps, 0, 0, 0, self.grid_length, \
                            self.grid_width, self.grid_height, collision_threshold):
                           continue

                        if n_id in closedset:
                            continue
                        # Otherwise if it is already in the open set
                        if n_id in openset:
                            if openset[n_id].cost > node.cost:
                                openset[n_id].cost = node.cost
                                openset[n_id].pind = c_id
                        else:
                            openset[n_id] = node

        end = rospy.Time().now().secs
        self.debugger_pub.publish(str(end - start))


        rx, ry, rz, t = self.calc_final_traj(ngoal, closedset, self.grid_reso) #this traj is reversed in time
        return [(rx[len(t) - 1 - i], ry[len(t) - 1 - i], rz[len(t) - 1 - i], \
            t[len(t) - 1 - i]) for i in range(len(t))]


        
    def calc_final_traj(self, ngoal, closedset, grid_reso):
        # generate final course
        rx, ry, rz, t = [ngoal.x * grid_reso], [ngoal.y * grid_reso], [ngoal.z * grid_reso], \
            [ngoal.tstamp]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(n.x * grid_reso)
            ry.append(n.y * grid_reso)
            rz.append(n.z * grid_reso)
            t.append(n.tstamp)
            pind = n.pind

        return rx, ry, rz, t



    def calc_h(self, gx, gy, gz, x, y, z):
        w = 10.0  # weight of heuristic
        d = w * math.sqrt((gx - x)**2 + (gy - y)**2 + (gz - z)**2)
        return d



    def verify_node(self, node, obmaps, minx, miny, minz, maxx, maxy, maxz, collision_threshold): #human obmap must constitute obmaps[0]
        if node.x < minx:
            return False
        elif node.y < miny:
            return False
        elif node.x >= maxx:
            return False
        elif node.y >= maxy:
            return False
        elif node.z < minz:
            return False
        elif node.z >= maxz:
            return False

        human_obmap = obmaps[0]
        if not self.verify_human(node, human_obmap, collision_threshold):
            return False

        for i in range(len(obmaps) - 1):
            obmap = obmaps[i + 1]
            if not self.verify_robot(node, obmap, collision_threshold):
                return False

        return True
        


    def verify_human(self, node, obmap, collision_threshold): #returns True if node is safe
        times = list(obmap.keys())
        if node.tstamp > times[len(times) - 1]: #planning time exceeds time obstacle exists
            last_time = times[len(times) - 1]
            obstacle_grid = obmap[last_time]
            if sum(self.integrator(node, obstacle_grid)) >= collision_threshold:
                return False
            else:
                return True
        else:
            for i in range(len(times)): #planning for a time that exists exactly in the obmap
                eps = 0.001
                if np.abs(node.tstamp - times[i]) < eps:
                    obstacle_grid = obmap[times[i]]
                    if sum(self.integrator(node, obstacle_grid)) >= collision_threshold:
                        return False
                    else:
                        return True


        prev_t = 0  #planning for a time that is in between time stamps in the obmap
        next_t = 0
        for i in range(len(times) - 1):
            if times[i] < node.tstamp and times[i + 1] > node.tstamp:
                prev_t = times[i]
                next_t = times[i + 1]
        low_grid = obmap[prev_t]
        high_grid = obmap[next_t]
        interpolated_obstacle_grid = np.zeros((self.grid_length, self.grid_width, self.grid_height))
        for i in range(len(low_grid)):
            for j in range(len(low_grid[0])):
                for k in range(len(low_grid[0][0])):
                    prev = low_grid[i][j][k]
                    next = high_grid[i][j][k]
                    curr = prev + (next - prev) * ((node.tstamp - prev_t) / (next_t - prev_t))
                    interpolated_obstacle_grid[i][j][k] = curr
 
        if sum(self.integrator(node, interpolated_obstacle_grid)) >= collision_threshold:
            return False
        else:
            return True



    def verify_robot(self, node, obmap, collision_threshold): #returns True if node is safe
        times = list(obmap.keys())
        if node.tstamp > times[len(times) - 1]: #planning time exceeds time obstacle exists
            last_time = times[len(times) - 1]
            obstacle_grid = obmap[last_time]
            if np.any(self.integrator(node, obstacle_grid)):
                return False
            else:
                return True
        else:
            for i in range(len(times)): #planning for a time that exists exactly in the obmap
                eps = 0.001
                if np.abs(node.tstamp - times[i]) < eps:
                    obstacle_grid = obmap[times[i]]
                    if np.any(self.integrator(node, obstacle_grid)):
                        return False
                    else:
                        return True

        prev_t = 0  #planning for a time that is in between time stamps in the obmap
        next_t = 0
        for i in range(len(times) - 1):
            if times[i] < node.tstamp and times[i + 1] > node.tstamp:
                prev_t = times[i]
                next_t = times[i + 1]
        before = self.integrator(node, obmap[prev_t])
        after = self.integrator(node, obmap[next_t])
        if np.any(before) or np.any(after): 
            return False
        else:
            return True




    def integrator(self, node, obstacle_grid): #returns obstacle grid cells to sum over
        integrate_over = []
        radius = math.ceil(self.robot_size + self.tracking_error_bound)
        cube_radius = math.sqrt(radius**2 + radius**2 + radius**2) #cube that circumscribes the sphere defined by teb
        for i in range(len(obstacle_grid)):
            for j in range(len(obstacle_grid[0])):
                for k in range(len(obstacle_grid[0][0])):
                    dist_to_node = math.sqrt((i - node.x)**2 + (j - node.y)**2 + (k - node.z)**2)
                    if dist_to_node <= cube_radius:
                        integrate_over.append(obstacle_grid[i][j][k])

        return integrate_over
        


    #def calc_index(self, node, xwidth, xmin, ymin):
    #    return (node.y - ymin) * xwidth + (node.x - xmin)

    def calc_index(self, node):
        return (node.x * 100) + (node.y * 10) + node.z


    def calculate_motion_cost(self, dx, dy, dz):
        return math.sqrt(math.fabs(dx) + math.fabs(dy) + math.fabs(dz))


    def calculate_motion_speed(self, dx, dy, dz):
        return math.sqrt(math.fabs(dx) + math.fabs(dy) + math.fabs(dz)) * self.speed




