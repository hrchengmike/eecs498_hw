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

    #compute the trajectory of robot given current pose and control input
    #returns trajectory
    #control = list [fx, fy, t] force and torque
    def computeTraj(self, control):
        px = self.x[0]
        py = self.x[1]
        theta = self.x[2]
        vx = self.x[3]
        vy = self.x[4]
        omega = self.x[5]
        fx = control[0]
        fy = control[1]
        t = control[2]
        ax = fx/self.m
        ay = fy/self.m
        beta = t/self.I
        dt = self.dt
        traj = []
        for i in range(int(self.dt_ctl/self.dt)):
            px = px + dt * vx
            py = py + dt * vy
            theta = theta + omega * dt
            vx = vx + ax * dt
            vy = vy + ay * dt
            omega = omega + beta * dt
            traj.append(np.array([px, py, theta]))
        self.x = np.array([px, py, theta, vx, vy, omega])
        return traj

    def computeControls(self, controls):
        traj = []
        for control in controls:
            self.executeTraj(self.computeTraj(control))

    #compute the goal position of robot given current pose and control input
    #returns end position
    #def computePos(control)


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

    hovercraft = hovercraft_class(robot, 1, 1, 0.001, 0.001, 3)
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

    raw_input("Press enter to exit...")
    env.Destroy()


