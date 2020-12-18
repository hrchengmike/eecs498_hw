#!/usr/bin/env python
from kinodynamic_rrt import *
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
    m = 5 # mass of hovercraft
    I = 5 # moment of inertia of hovercraft
    dt = 0.01 #time increment during numerical integration
    dt_vis = 0.01 # time increment during visualization
    dt_ctl = 1.2 # time gap of adjacent control 0.8, 2.4
    #controls = [[1, 0, 0],[0, 0, 1]]
    #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]]
    #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0.5, 0, 0], [-0.5, 0, 0], [0, 0.5, 0], [0, -0.5, 0]] # set of control
    #controls = [[2, 0, 0], [-2, 0, 0], [0, 2, 0], [0, -2, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0.5, 0, 0], [-0.5, 0, 0], [0, 0.5, 0], [0, -0.5, 0]] # set of control
    controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0.5, 0, 0], [-0.5, 0, 0], [0, 0.5, 0], [0, -0.5, 0], [0.25, 0, 0], [-0.25, 0, 0], [0, 0.25, 0], [0, -0.25, 0], [0.75, 0, 0], [-0.75, 0, 0], [0, 0.75, 0], [0, -0.75, 0]] # set of control
    #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0]]

    #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [1, 1, 0], [1, -1, 0], [-1, 1, 0], [-1, -1, 0],[0.5, 0, 0], [-0.5, 0, 0], [0, 0.5, 0], [0, -0.5, 0], [0.5, 0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0], [-0.5, -0.5, 0]] # set of control
    #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0.5, 0, 0], [-0.5, 0, 0], [0, 0.5, 0], [0, -0.5, 0],] # set of control
    #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [2, 0, 0], [-2, 0, 0], [0, 2, 0], [0, -2, 0]]
    hovercraft = hovercraft_class(robot, start_config, m, I, dt, dt_vis, dt_ctl)
    hovercraft.setRobotPose(start_config[:3])
    time.sleep(0.1)

    with env:
        #run kinodynamic rrt
        duration, traj, traj_time, full_len = kinodynamic_rrt(start_config, goal_config, e, bias, n, m, I, dt, dt_vis, dt_ctl, controls, env, robot, handles, True)
    print "time: ", duration
    print "traj_time", traj_time
    print "traj_len", full_len
    hovercraft.executeTraj(traj, 0.2)


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
