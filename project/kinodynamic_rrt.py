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

#sets the robot pose and show in gui, pose is in numpy array [x, y, theta]
def setRobotPose(robot, pose):
    theta = pose[2]
    T = np.array([[np.cos(theta), -np.sin(theta), 0, pose[0]],
                   [np.sin(theta),  np.cos(theta), 0, pose[1]],
                   [         0,           0,       1,       0],
                   [         0,           0,       0,       1]])
    robot.SetTransform(T)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from environment XML file
    env.Load('hovercraft_env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]
    print "Robot in collision with environment?", env.CheckCollision(robot)

    for i in arange(0.0, 1.0, 0.004):
        setRobotPose(robot, np.array([i, 0, 0]))
        time.sleep(0.01)
    for i in arange(0.0, np.pi/2, 0.004):
        setRobotPose(robot, np.array([1, 0, i]))
        time.sleep(0.01)
    for i in arange(0.0, 1.0, 0.004):
        setRobotPose(robot, np.array([1, i, np.pi/2]))
        time.sleep(0.01)

    raw_input("Press enter to exit...")
    env.Destroy()


