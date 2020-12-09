#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy as np
from rrt import *

#### YOUR IMPORTS GO HERE ####

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [joint1_i, joint2_i, joint3_i,...]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeActiveDOFTrajectory(traj,robot)#,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

class node:
    def __init__ (self, x, parent=None, ctl=None):
        #configuration saved as numpy array
        # state vector x = [px, py, theta, vx, vy, omega]
        self.x = x
        # what control applied to get to current state from parent node
        self.ctl = ctl
        if not parent == None:
            self.parent = parent
        self.children = []

    def add_child (self, child):
        assert isinstance(child, node)
        self.children.append(child)
        if node.plot_path is True:
            self.plot_node_traj(child)

    def plot_node_traj(self, skip = 20):
           for i, pt in enumerate(self.traj):
               '''
               node.handles.append(node.env.drawlinestrip(points =hstack([T_cur[0:3, 3:4].T, T_par[0:3, 3:4].T]), linewidth=3.0, colors=array(((1,0,0),(0,0,1)))))
               '''
               if i % skip == 0:
                   node.handles.append(node.env.plot3(points=array(pt),
                                           pointsize=5.0,
                                           colors=array(((0,0,1,0.2)))))

    def computeNextState(self, control):
        px = self.x[0]
        py = self.x[1]
        theta = self.x[2]
        vx = self.x[3]
        vy = self.x[4]
        omega = self.x[5]
        fx = control[0]
        fy = control[1]
        t = control[2]
        ax = fx/node.m
        ay = fy/node.m
        beta = t/node.I
        dt = node.dt
        for i in range(int(node.dt_ctl/node.dt)):
            px = px + dt * vx
            py = py + dt * vy
            theta = theta + omega * dt
            vx = vx + ax * dt
            vy = vy + ay * dt
            omega = omega + beta * dt
        next = np.array([px, py, theta, vx, vy, omega])
        return next

    #return the trajectory of node, a list of nparrays
    #from parent to current node
    def comp_traj(self):
        px = self.parent.x[0]
        py = self.parent.x[1]
        theta = self.parent.x[2]
        vx = self.parent.x[3]
        vy = self.parent.x[4]
        omega = self.parent.x[5]
        fx = self.ctl[0]
        fy = self.ctl[1]
        t = self.ctl[2]
        ax = fx/node.m
        ay = fy/node.m
        beta = t/node.I
        dt = node.dt_vis
        traj = []
        for i in range(int(node.dt_ctl/dt)):
            px = px + dt * vx
            py = py + dt * vy
            theta = theta + omega * dt
            vx = vx + ax * dt
            vy = vy + ay * dt
            omega = omega + beta * dt
            traj.append(np.array([px, py, theta]))
            self.traj = traj

class hovercraft_class:
    def __init__(self, robot, m, I, dt, dt_vis, dt_ctl):
        self.robot = robot
        self.m = m #mass
        self.I = I #moment of inertia
        self.dt = dt #time increment during integration
        self.dt_vis = dt_vis # time increment during visualization
        self.dt_ctl = dt_ctl # time increment between two control input
        self.x = np.array([-1.0, -2.0, 0, 0, 0.5, 0])
         # state vector x = [px, py, theta, vx, vy, omega]

    #execute traj which is a list of np arrays [q1, ..., qn], qi=[x, y, theta]
    #dt is the time between adjacent qi
    def executeTraj(self, traj):
        for i in range(len(traj)):
            self.setRobotPose(self.robot, traj[i])
            time.sleep(self.dt_vis)

    #sets the robot pose and show in gui, pose is in numpy array [x, y, theta]
    def setRobotPose(self, robot, pose):
        theta = pose[2]
        T = np.array([[np.cos(theta), -np.sin(theta), 0, pose[0]],
                       [np.sin(theta),  np.cos(theta), 0, pose[1]],
                       [         0,           0,       1,       0],
                       [         0,           0,       0,       1]])
        self.robot.SetTransform(T)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from environment XML file
    env.Load('hovercraft_env.xml')
    time.sleep(2.0)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    print "Robot in collision with environment?", env.CheckCollision(robot)

    handles = []
    hovercraft = hovercraft_class(robot, 1, 1, 0.001, 0.005, 3)
    node.handles = handles
    node.robot = robot
    node.env = env
    node.m = hovercraft.m
    node.I = hovercraft.I
    node.dt = hovercraft.dt #time increment during integration
    node.dt_vis = hovercraft.dt_vis # time increment during visualization
    node.dt_ctl = hovercraft.dt_ctl
    node.plot_path = True

    root = node(np.array([-1.0, -2.0, 0, 0, 0, 0]))
    controls = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    control = [1, 1, 0]
    child1 = node(root.computeNextState(control), root, control)
    child1.comp_traj()
    hovercraft.executeTraj(child1.traj)
    print child1.parent.x
    print child1.x, child1.ctl
    child1.plot_node_traj()
    '''
    traj = []
    for i in arange(0.0, 1.0, 0.001):
        traj.append(np.array([i, 0, 0]))
    for i in arange(0.0, 1.0, 0.001):
        traj.append(np.array([1, 0, i]))
    for i in arange(0.0, 1.0, 0.001):
        traj.append(np.array([1, i, np.pi/2]))
    control = [1, 0, 0]
    #traj2 = hovercraft.computeTraj(control)
    #print traj2[-1]
    controls = [[1,0,0], [-1,0,0],[-1,0,0],[1,0,0]]
    hovercraft.computeControls(controls)
    #hovercraft.executeTraj(traj2)
    '''
    raw_input("Press enter to exit...")
    env.Destroy()


