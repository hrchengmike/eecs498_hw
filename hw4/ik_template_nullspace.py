#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import numpy
#### YOUR IMPORTS GO HERE ####
#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['torso_lift_joint','l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([0.24,1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

#set active DOF values from a numpy matrix
def SetActiveDOFValuesNPMatrix(robot,qmat):
    qo = [q.item(i) for i in range(0,qmat.shape[1])]
    robot.SetActiveDOFValues(qo)


#returns the end effector transform in the world frame
def GetEETransform(robot,activedofvalues=None):
    if activedofvalues != None:
        robot.SetActiveDOFValues(activedofvalues);
    manip = robot.GetActiveManipulator()
    return manip.GetEndEffectorTransform()

#returns the joint axis in the world frame
def GetJointAxis(robot,jointname):
    return robot.GetJoint(jointname).GetAxis(0)

#returns the joint position in the world frame
def GetJointPosition(robot,jointname):
    return robot.GetJoint(jointname).GetAnchor()

def GetTranslationJacobian(robot,jointnames):
    J = numpy.zeros((3,robot.GetActiveDOF()))
    ### YOUR CODE HERE ###
    #current end effector position
    eef_pos = GetEETransform(robot)[0:3, 3:4].T
    i = 0 # iterator in for loop
    #calculate each column of Jacobian as th joint's axis cross product r
    #where r is vector from joint position to end effector position
    for name in jointnames:
        axis = GetJointAxis(robot, name)
        joint_pos = GetJointPosition(robot, name)
        J[:,i:i+1] = cross(axis, eef_pos - joint_pos).T
        i = i+1
    ### YOUR CODE HERE ###
    return J

def GetJpinv(J, l = 0.001):
    ### YOUR CODE HERE ###
    Jpinv = []
    I = identity(J.shape[0])
    #Jpinv =linalg.inv(dot(J.T, J)+l**2*I).dot(J.T)
    Jpinv =J.T.dot(linalg.inv(dot(J, J.T)+l**2*I))
    ### YOUR CODE HERE ###
    return Jpinv


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    env.Reset()
    # load a scene from environment XML file
    env.Load('pr2only.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms and raise torso
    tuckarms(env,robot);

    #set start config
    robot.SetActiveManipulator('leftarm')
    jointnames =['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint']
    robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])

    targets = [[-0.15070158,  0.47726995,  1.56714123],
           [-0.36535318,  0.11249, 1.08326675],
           [-0.56491217,  0.011443, 1.2922572 ],
           [-1.07012697,  0.81909669,  0.47344636],
           [-1.11050811,  0.97000718,  1.31087581]]
    doflimits = robot.GetActiveDOFLimits() #make sure q doesn't go past these limits
    q = numpy.zeros((1,robot.GetActiveDOF())) #start at this configuration
    with env:
        start = time.clock()
        handles = [] #graphics handles for plotting
        SetActiveDOFValuesNPMatrix(robot,q)

        ### YOUR CODE HERE ###
        target = targets[1] ###pick your target here
        #draw the target point in blue
        handles.append(env.plot3(points=array(target), pointsize=15.0, colors=array((0,0,1)) ))
        e = 0.01 # euclidean distance between end effector and goal before stop
        step = 0.2 #the max norm of delta q of each iteration
        lower,upper = robot.GetActiveDOFLimits()
        while True:
            eef_pos = GetEETransform(robot)[0:3, 3:4].T
            # vector pointing from end effector(eef) to target
            dx =target - eef_pos
            # target reached by eef
            if(linalg.norm(dx) < e):
                break
            dq = GetJpinv(GetTranslationJacobian(robot, jointnames)).dot(dx.T)
            #Rescale dq s.t. norm is step
            if(linalg.norm(dq) > step):
                dq = step * dq/linalg.norm(dq)
            q = q + dq.T
            #check whether each joint is within joint limit
            #if not, set to joint limit
            for i in range(7):
                if q[0,i] < lower[i]:
                    q[0,i] = lower[i]
                if q[0,i] > upper[i]:
                    q[0,i] = upper[i]
            robot.SetActiveDOFValues(q[0,:])
        end = time.clock()
        print "Time: ", end - start
        print "Joint angles when reaching the target: ", q[0,:]
        ### YOUR CODE HERE ###

    robot.GetController().SetDesired(robot.GetDOFValues())
    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
