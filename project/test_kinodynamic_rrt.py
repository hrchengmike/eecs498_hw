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
    n = 10000 # total number of iteration
    m = 5 # mass of hovercraft
    I = 5 # moment of inertia of hovercraft
    dt = 0.01 #time increment during numerical integration
    dt_vis = 0.01 # time increment during visualization
    dt_ctl = 1.2 # time gap of adjacent control 0.8, 1.5 m = 2, I= 2, revwese YU
    #controls = [[1, 0, 0],[0, 1, 0]]

    tot_it = 10
    num_angles = [40]
    result_comp_time = np.zeros((len(num_angles),tot_it))
    result_path_time = np.zeros((len(num_angles),tot_it))
    for i, num_angle in enumerate(num_angles):
        for it in range(tot_it):
            print "num_angles:", num_angle
            print "m:", m
            print "dt_ctl:", dt_ctl
            print "i : {}, it: {}".format(i,it)
            controls = []
            for j in range(num_angle):
                controls.append([cos(2*pi/num_angle*j), sin(2*pi/num_angle*j), 0])
            #controls = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]]
            hovercraft = hovercraft_class(robot, start_config, m, I, dt, dt_vis, dt_ctl)
            hovercraft.setRobotPose(start_config[:3])
            time.sleep(0.1)
            #run kinodynamic rrt
            with env:
                duration, traj, traj_time, full_len= kinodynamic_rrt(start_config, goal_config, e, bias, n, m, I, dt, dt_vis, dt_ctl, controls, env, robot, handles, False)
            print "time: ", duration
            print "traj_time", traj_time
            print "traj_len", full_len
            result_comp_time[i][it] = duration
            result_path_time[i][it] = traj_time
            #hovercraft.executeTraj(traj, 0.5)
            handles = []
            print result_path_time
            print result_comp_time
            print result_path_time.min(axis = 1), np.mean(result_path_time, axis=1),result_path_time.max(axis = 1)
            print result_comp_time.min(axis = 1), np.mean(result_comp_time, axis=1),result_comp_time.max(axis = 1)
            #time.sleep(2)
    #write result to file
    f= open("data_direction_m_{}_ctl_{}_it{}_40.txt".format(m, dt_ctl,tot_it),"w+")
    for i, num_angle in enumerate(num_angles):
        f.write(str(num_angle)+" & ")
        f.write("{:.2f}".format(result_path_time[i,:].min())+" & ")
        f.write("{:.2f}".format(np.mean(result_path_time[i,:]))+ " & ")
        f.write("{:.2f}".format(result_path_time[i,:].max())+" & ")
        f.write("{:.2f}".format(result_comp_time[i,:].min())+" & ")
        f.write("{:.2f}".format(np.mean(result_comp_time[i,:])) + " & ")
        f.write("{:.2f}".format(result_comp_time[i,:].max())+" \\ \n")
    f.write("raw data \n")
    f.write("result_path_time\n")
    f.write(str(result_path_time)+"\n")
    f.write("result_comp_time\n")
    f.write(str(result_comp_time))
    f.close()

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
