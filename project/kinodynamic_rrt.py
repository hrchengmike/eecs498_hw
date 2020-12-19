#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import numpy as np
from math import *
import Queue
from openravepy import *

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class node:
    def __init__ (self, x, parent=None, ctl=None):
        #configuration saved as numpy array
        # state vector x = [px, py, theta, vx, vy, omega]
        self.x = x
        # what control applied to get to current state from parent node
        #control is numpy array [fx, fy, torque]
        self.ctl = ctl
        if not parent == None:
            self.parent = parent
        self.children = []

    # add child node to current node, can also plot trajectory from node to child
    def add_child (self, child):
        assert isinstance(child, node)
        self.children.append(child)

    # plot the trajectory of self.traj, trajectory from parent to current node, which is computed from comp_traj
    def plot_node_traj(self, skip = 5, type = 'pt'):
        for i, pt in enumerate(self.traj):
            if i == 0:
                prev_pt = pt
            if i % skip == 0:
                if type == 'pt':
                    #node.handles.append(node.env.plot3(points=array((pt[0], pt[1], 0.05)), pointsize=2.0, colors=array(((0,0,1,0.2)))))
                    node.handles.append(node.env.drawlinestrip(points =hstack([array((pt[0], pt[1], 0.05)), array((prev_pt[0], prev_pt[1], 0.05))]), linewidth=3.0,colors=array(((0,0,1),(0,0,1)))))
                    prev_pt = pt
                if type == 'line':
                    node.handles.append(node.env.drawlinestrip(points =hstack([array((pt[0], pt[1], 0.05)), array((prev_pt[0], prev_pt[1], 0.05))]), linewidth=4.0,colors=array(((0,0,0),(0,0,0)))))
                    prev_pt = pt

        #node.handles.append(node.env.plot3(points=array((self.x[0], self.x[1], 0.05)), pointsize=3.0, colors=array(((1,0,0,0.2)))))
    # compute next state of current node given control
    # control is list or numpy array
    # return none when robot collides on the way of cur->next
    def computeNextState(self, control, collision_skip = 10):
        px = self.x[0]
        py = self.x[1]
        theta = self.x[2]
        vx = self.x[3]
        vy = self.x[4]
        omega = self.x[5]
        #force in local frame, transform to global frame
        fx_local = control[0]
        fy_local = control[1]

        t = control[2]

        beta = t/node.I
        dt = node.dt
        for i in range(int(node.dt_ctl/node.dt)):
            px = px + dt * vx
            py = py + dt * vy
            theta = theta + omega * dt

            fx = fx_local * np.cos(theta) - fy_local * np.sin(theta)
            fy = fx_local * np.sin(theta) + fy_local * np.cos(theta)
            ax = fx/node.m
            ay = fy/node.m

            vx = vx + ax * dt
            vy = vy + ay * dt
            omega = omega + beta * dt
            #every collision_skip, check whether robot is in collision
            if i % collision_skip == 0:
                node.hovercraft.setRobotPose([px, py, theta])
                if node.env.CheckCollision(node.hovercraft.robot):
                    return None
        next = np.array([px, py, theta, vx, vy, omega])
        return next

    #return the trajectory of node, a list of nparrays
    #from parent to current node
    #timestep dt_vis
    #REQUIRES: not root node, self.parent, self.ctl
    def comp_traj(self):
        if self.parent == None:
            print "root node, no trajectory from parent"
            return
        #path length of the trajectory from current to parent node
        self.path_len = 0
        px = self.parent.x[0]
        py = self.parent.x[1]
        theta = self.parent.x[2]
        vx = self.parent.x[3]
        vy = self.parent.x[4]
        omega = self.parent.x[5]
        #force in local frame, transform to global frame
        fx_local = self.ctl[0]
        fy_local = self.ctl[1]

        t = self.ctl[2]

        beta = t/node.I
        dt = node.dt_vis
        traj = []
        for i in range(int(node.dt_ctl/dt)+1):
            px = px + dt * vx
            py = py + dt * vy
            theta = theta + omega * dt

            fx = fx_local * np.cos(theta) - fy_local * np.sin(theta)
            fy = fx_local * np.sin(theta) + fy_local * np.cos(theta)
            ax = fx/node.m
            ay = fy/node.m

            vx = vx + ax * dt
            vy = vy + ay * dt
            omega = omega + beta * dt
            traj.append(np.array([px, py, theta]))
            self.path_len += sqrt((vx)**2+(vy)**2)*dt
        self.traj = traj

class hovercraft_class:
    def __init__(self, robot, x, m, I, dt, dt_vis, dt_ctl):
        self.robot = robot
        self.m = m #mass
        self.I = I #moment of inertia
        self.dt = dt #time increment during integration
        self.dt_vis = dt_vis # time increment during visualization
        self.dt_ctl = dt_ctl # time increment between two control input
        self.x = x
         # state vector x = [px, py, theta, vx, vy, omega]

    #execute traj which is a list of np arrays [q1, ..., qn], qi=[x, y, theta]
    #dt is the time between adjacent qi
    def executeTraj(self, traj, time_scale = 1):
        for i in range(len(traj)):
            self.setRobotPose(traj[i])
            time.sleep(self.dt_vis*time_scale)

    #sets the robot pose and show in gui, pose is in numpy array [x, y, theta]
    def setRobotPose(self, pose):
        theta = pose[2]
        T = np.array([[np.cos(theta), -np.sin(theta), 0, pose[0]],
                       [np.sin(theta),  np.cos(theta), 0, pose[1]],
                       [         0,           0,       1,       0],
                       [         0,           0,       0,       1]])
        self.robot.SetTransform(T)

#returns a node of random configuration in the free space with probability (1-bias), with a probability of bias return goal node
def random_config (env, hovercraft, goal, bias, x_lim = [-5, 5], y_lim = [-5, 5], v_lim = [-0.5, 0.5], omega_lim = [-0.5, 0.5]):
    rand = random.rand()
    #set goal for probability of bias
    if rand < bias:
        return goal
    #randomly sample in free space
    else:
        #get joint limits and randomly sample in c-space
        state_range = np.vstack((np.array(x_lim), np.array(y_lim), np.array([-np.pi, np.pi]), np.array(v_lim), np.array(v_lim), np.array(omega_lim))).T
        lower = state_range[0,:]
        upper = state_range[1,:]
        while True:
            q_rand = random.rand(len(lower))*(upper-lower)+lower
            hovercraft.setRobotPose(q_rand[:3])
            if not env.CheckCollision(hovercraft.robot):
                return node(q_rand)

#use BFS to find the nearest node to rand node in the tree
def find_near(root, rand, goal):
    q = Queue.Queue()
    q.put(root)
    #stores the minimum distance from node to rand in the tree
    min_dist = dist(root, rand)
    min_dist_node = root
    while not q.empty():
        cur = q.get()
        for child in cur.children:
            q.put(child)
        if dist(cur, rand)< min_dist:
            min_dist_node = cur
            min_dist = dist(cur, rand)
    return min_dist_node

#TODO: weighted distance
#find the euclidean distance between the states of two nodes a, b
def dist(a, b):
    ans = 0
    #apart from angle theta, all states have R1 topology
    for i in range(6):
        if not i == 2:
            ans = ans + (a.x[i] - b.x[i]) ** 2
    #the orientation theta has S1 topology
    ans = ans + angle_diff(a.x[2], b.x[2])**2
    return sqrt(ans)

#a and b are angles in radians within [-pi, pi], angle_diff returns the angle difference within [-pi, pi]
def angle_diff(a, b):
    diff = a - b
    while(diff < -pi):
        diff = diff + 2*pi
    while(diff > pi ):
        diff = diff - 2*pi
    return diff

#extend from node near to node rand by trying all controls and finding the nearest node
#returns the extended node, if robot collides under all controls, return Nonw
def extend(near, rand):
    next_states = []
    #apply all controls
    for control in node.controls:
        next_states.append(near.computeNextState(control))
    #find nearest state over all controls
    min_dist = 1000.0
    min_dist_id = None
    for i, state in enumerate(next_states):
        if state is not None:
            if dist(node(state), rand) < min_dist:
                min_dist = dist(node(state), rand)
                min_dist_id = i
    # all controls collide, return None
    if min_dist_id is None:
        return None
    new = node(next_states[min_dist_id], near, node.controls[min_dist_id])
    return new

def full_path(root, cur_node, plot = True):
    traj = []
    #full path length
    path_full_len= 0
    #number of control time step for the full path
    len = 0
    while True:
        len = len + 1
        if cur_node.plot_all_branches is False:
            cur_node.comp_traj()
        if plot is True:
            cur_node.plot_node_traj(type = 'line')
        path_full_len += cur_node.path_len
        traj = cur_node.traj + traj
        cur_node = cur_node.parent
        if cur_node.parent == root:
            node.traj_time = len * node.dt_ctl
            node.path_len = path_full_len
            break
    return traj

# run kinodynamic rrt and return the trajectory planned
def kinodynamic_rrt(start_config, goal_config, e, bias, n,  m, I, dt, dt_vis, dt_ctl, controls, env, robot, handles, plot_all_branches = True):
    #initialize variable
    start = time.time()
    hovercraft = hovercraft_class(robot, start_config, m, I, dt, dt_vis, dt_ctl)
    node.handles = handles
    node.robot = robot
    node.env = env
    node.m = m
    node.I = I
    node.dt = dt #time increment during integration
    node.dt_vis = dt_vis # time increment during visualization
    node.dt_ctl = dt_ctl
    node.plot_all_branches = plot_all_branches
    node.controls = controls
    node.handles = handles
    node.robot = robot
    node.env = env
    node.hovercraft = hovercraft
    root = node(array(start_config), None, None)
    goal = node(array(goal_config))
    min_dist2goal = 100
    for k in range(n):
        rand = random_config (env, hovercraft, goal, bias)
        near = find_near(root, rand, goal)
        new = extend(near, rand)
        if new is not None:
            near.add_child(new)
            if node.plot_all_branches is True:
                new.comp_traj()
                new.plot_node_traj()
            dist2goal = np.sqrt((new.x[0] - goal.x[0])**2+(new.x[1] - goal.x[1])**2)
            if dist2goal < min_dist2goal:
                min_dist2goal = dist2goal
                #print "iteration: ", k,  " min_dist2goal: ", min_dist2goal
            if dist2goal < e:
                node.handles = []
                handles = []
                time.sleep(0.5)
                print "goal reached"
                end = time.time()
                return end-start, full_path(root, new), node.traj_time, node.path_len




