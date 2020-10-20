#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import math

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    #### YOUR CODE HERE ####

    # print
    with env:
        l = 0.7 #length of table
        w = 1.2 #width of table
        h = 1.2 #height of table
        z_offset = -0.5; #shift the block in z direction

        #generate an array of vertices in local frame
        v = zeros((8,3))
        m = 0
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    v [m, :] = array([l/2*i, w/2*j, h/2*k])
                    m = m + 1

        # For every pair of vertices, check whether the line segment is an edge,
        # if yes, push into numpy array
        edges = zeros((48,3))
        m = 0
        for i in v:
            for j in v:
                sum = i+j
                zeros = int(sum[0] == 0) + int(sum[1] == 0) + int(sum[2] == 0)
                if zeros == 1:
                    edges [m*2:m*2+2,:] = [i,j]
                    m = m + 1
        # Generate plot for each table in the environment
        handles = []
        count = 0
        for body in env.GetBodies():
            count = count +1
            # For bodies other than table, do nothing
            if count < 2 or count > 7:
                continue
            T = body.GetTransform()
            R = T[0:3,0:3]
            t = T[0:3,3:4]

            # for every edge, transform from local to global coordinate and paint
            for m in range(24):
                k = edges[2*m:2*m+2,:]+[0, 0, z_offset]
                globalCoordinate = (dot(R, k.T)+t).T
                handles.append(env.drawlinestrip(points = globalCoordinate, linewidth=3.0,colors=array(((1,0,0),(1,0,0)))))

        #Print 35 blue points in a circle that encompasses the scene
        r= 5.0
        for i in range(35):
            theta = i*(pi*2/35)
            handles.append(env.plot3(points=array((r * cos(theta), r * sin(theta),0)),pointsize=15.0,colors=array((0,0,1))))

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()
