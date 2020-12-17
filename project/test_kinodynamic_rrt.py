#!/usr/bin/env python
from kinodynamic_rrt import *
from math import *
import time
if __name__ == "__main__":

    #initialize environment
    env = Environment()
    env.SetViewer('qtcoin')
    # env.SetDebugLevel(1) #comment this in to turn off openrave warnings
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)
    env.Reset()

    # load a scene from environment XML file
    env.Load('hovercraft_env2.xml')
    time.sleep(0.5)

    #load robot
    robot = env.GetRobots()[0]
    #set variables
    handles = []
    start_config = np.array([-4.5, -4.5, 0, 0, 0, 0])
    goal_config = np.array([4.5, 4.5, 0, 0, 0, 0])
    e = 0.4
    bias = 0.1 # probability of selecting goal as the random config
    n = 5000 # total number of iteration
    m = 2 # mass of hovercraft
    I = 2 # moment of inertia of hovercraft
    dt = 0.01 #time increment during numerical integration
    dt_vis = 0.05 # time increment during visualization
    dt_ctl = 0.8 # time gap of adjacent control 0.8, 2.4
    #controls = [[1, 0, 0],[0, 1, 0]]

    for i in [4,6,8,10]:
        controls = []
        for j in range(i):
            controls.append([cos(2*pi/i*j), sin(2*pi/i*j), 0])
        print controls
        #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]]
        hovercraft = hovercraft_class(robot, start_config, m, I, dt, dt_vis, dt_ctl)
        hovercraft.setRobotPose(start_config[:3])
        time.sleep(0.1)
        #run kinodynamic rrt
        with env:
            traj = kinodynamic_rrt(start_config, goal_config, e, bias, n, m, I, dt, dt_vis, dt_ctl, controls, env, robot, handles, False)
            print len(traj)
        hovercraft.executeTraj(traj, 0.5)

    #execute path
    '''
    root = node(np.array([-1.0, -2.0, 0.5, np.cos(0.5), np.sin(0.5), 0]))
    control = [0, 1, 0]
    child1 = node(root.computeNextState(control), root, control)
    child1.comp_traj()
    hovercraft.executeTraj(child1.traj)
    print child1.parent.x
    print child1.x, child1.ctl
    child1.plot_node_traj()
'''
    raw_input("Press enter to exit...")
    env.Destroy()
