from numpy import *
from math import *
import time
import openravepy
from Queue import PriorityQueue

#defines a basic node class
class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in
        self.g = 0;
        self.h = 0;
        self.f = 0;

    def __eq__(self, other):
        return round(self.x, self.p) == round(other.x, self.p) and round(self.y, self.p) == round(other.y, self.p)

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid, "g_costs", self.g, "h_costs", self.h, "f_costs", self.f

    def computeCosts(self, parent):
        self.g = parent.g + dist(self, parent)
        self.h = dist(self, self.goal)
        self.f = self.g + self.h

def dist(a, b):
    return sqrt((a.x - b.x)**2 + (a.y - b.y)**2)

def path(id, tree, handles, env):
    path = []
    cur = tree[id]
    while not cur.id == 0:
        ar =[cur.x, cur.y, cur.theta]
        path.append(ar)
        #path.append([cur.x, cur.y, cur.theta])
        prev = cur
        cur = tree [cur.parentid]
        handles.append(env.plot3(points=array((cur.x, cur.y, 0.01)),pointsize=12.0,colors=array((0,0,0))))
        handles.append(env.drawlinestrip(points = array([[prev.x, prev.y, 0.01],[cur.x, cur.y, 0.01]]), linewidth=7.0,colors=array(((0,0,0),(0,0,0)))))
    path.reverse()
    return path


# all neighbors of current node that is in the map and not in collision
# print neighbors in collision with red point
# print neighbors not in collision with blue point
def kids(cur, connect, step, handles, robot, env):
    kids = []
    # 4-connected
    if connect == 4 or connect == 8:
        for theta in arange(0, 2*pi, pi/2):
            x = cur.x + step * cos(theta)
            y = cur.y + step * sin(theta)
            T = robot.GetTransform()
            T[0:3,3:4]=array([[x],[y],[0.01]])
            T[0:3,0:3]=array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0],[0, 0, 1]])
            robot.SetTransform(T)
            #if not in robot not in collision with environment at this pose
            if not env.CheckCollision(robot):
                kid = Node(x, y, theta, 0, 0)
                kids.append(kid)
                handles.append(env.plot3(points=array((x, y, 0.01)),pointsize=10.0,colors=array((0,0,1))))
            else:
                handles.append(env.plot3(points=array((x, y, 0.01)),pointsize=10.0,colors=array((1,0,0))))
    if connect == 8:
         for theta in arange(pi/4, 2*pi, pi/2):
            x = cur.x + step * sqrt(2) * cos(theta)
            y = cur.y + step * sqrt(2) * sin(theta)
            T = robot.GetTransform()
            T[0:3,3:4]=array([[x],[y],[0.01]])
            T[0:3,0:3]=array([[cos(theta), -sin(theta), 0], [sin(theta), cos(theta), 0],[0, 0, 1]])
            robot.SetTransform(T)
            #if not in robot not in collision with environment at this pose
            if not env.CheckCollision(robot):
                kid = Node(x, y, theta, 0, 0)
                kids.append(kid)
                handles.append(env.plot3(points=array((x, y, 0.01)),pointsize=10.0,colors=array((0,0,1))))
            else:
                handles.append(env.plot3(points=array((x, y, 0.01)),pointsize=10.0,colors=array((1,0,0))))
    if not (connect == 4 or connect == 8):
        print "Wrong connect input, input 4 for 4-connected and 8 for 8-connected"
    return kids

def astar(startconfig, goalconfig, step, connect, env, robot, handles):
    with env:
        id = 1
        p = 2
        Node.p = p
        open = PriorityQueue()
        # closedlist is a dictionary mapping from rounded (x,y) to the node
        # stores all visited node, enables fast check whether a node is visited by its coordinates
        close = {}
        # tree is where all visited nodes are stored, tree[i] stores node with id = i, enables fast path reconstruction, tree[1] is start node,
        tree = []

        goal = Node(goalconfig[0], goalconfig[1], goalconfig[2], 0, 0)
        Node.goal = goal
        start = Node(startconfig[0], startconfig[1], startconfig[2], 0, 0)
        start.computeCosts(start)
        open.put((start.f, start))
        close[(round(start.x, p), round(start.y, p))] = 1
        tree.append(start)
        while not open.empty():
            cur_pri = open.get()
            cur = cur_pri[1]
            #print "========================================================="
            #print "Poped from openlist:"
            #cur_pri[1].printme()
            #print "Priority:", next_item[0]
            #next_item[1].printme()
            if dist(cur, goal) < 0.05:
                return path(cur.id, tree, handles, env)
            #print "all kids:"
            for kid in kids(cur, connect, step, handles, robot, env):
                if not close.has_key((round(kid.x, p), round(kid.y, p))):
                    #print "this kid's id:", id, "parent's id:", cur.id
                    new = Node(kid.x, kid.y, kid.theta, id, cur.id)
                    new.computeCosts(cur)
                    #print "new node"
                    #new.printme()
                    tree.append(new)
                    id = id + 1

                    open.put((new.f, new))
                    close[(round(kid.x, p), round(kid.y, p))] = 1
        print "No Solution Found by A*"
