from numpy import *
import math
import time
import openravepy

def astar(env, handles):
    print "hello"
    r= 5.0
    for i in range(35):
        theta = i*(pi*2/35)
        handles.append(env.plot3(points=array((r * cos(theta), r * sin(theta),0)),pointsize=15.0,colors=array((0,0,1))))
