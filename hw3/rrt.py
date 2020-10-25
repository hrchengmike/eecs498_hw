#rrt
from numpy import *
from math import *
import time
import Queue
from openravepy import *
from rrt_template import *

class node:
    def __init__ (self, q, parent=None):
        #configuration saved as numpy array
        self.q = q
        if not parent == None:
            self.parent = parent
        self.children = []
    def add_child (self, child):
        assert isinstance(child, node)
        self.children.append(child)
        T_cur = GetEETransform(node.robot, child.q)
        if len(self.children) == 1:
            T_par = GetEETransform(node.robot, self.q)
            node.handles.append(node.env.drawlinestrip(points =hstack([T_cur[0:3, 3:4].T, T_par[0:3, 3:4].T]), linewidth=3.0, colors=array(((1,0,0),(0,0,1)))))
        else:
            T_prev = GetEETransform(node.robot, self.children[len(self.children)-2].q)
            node.handles.append(node.env.drawlinestrip(points =hstack([T_cur[0:3, 3:4].T, T_prev[0:3, 3:4].T]), linewidth=3.0,colors=array(((1,0,0),(0,0,1)))))

    def __repr__(self):
        print "Configuration: ", self.q
        print "Children:"
        for child in children:
            print child.q

#returns a node of random configuration in the free space with probability (1-bias), with a probability of bias return goal node
def random_config (env, robot, goal, bias):
    rand = random.rand()
    #set goal for probability of bias
    if rand < bias:
        return goal
    #randomly sample in free space
    else:
        #get joint limits and randomly sample in c-space
        lower,upper = robot.GetActiveDOFLimits()
        while True:
            q_rand = random.rand(len(lower))*(upper-lower)+lower
            # joint [4] is of S1 topology
            q_rand[4] = -pi + 2*pi*random.rand()
            robot.SetActiveDOFValues(q_rand)
            if not env.CheckCollision(robot):
                return node(array(q_rand))

#use BFS to find the nearest node to rand node in the tree
def find_near(root, rand, goal):
    q = Queue.Queue()
    q.put(root)
    #stores the minimum distance from node to rand in the tree
    if rand == goal:
        min_dist = dist_w(root, rand)
        min_dist_node = root
        while not q.empty():
            cur = q.get()
            for child in cur.children:
                q.put(child)
            if dist_w(cur, rand)< min_dist:
                min_dist_node = cur
                min_dist = dist_w(cur, rand)
    else:
        min_dist = dist(root, rand)
        min_dist_node = root
        while not q.empty():
            cur = q.get()
            for child in cur.children:
                q.put(child)
            if dist(cur, rand)< min_dist:
                min_dist_node = cur
                min_dist = dist(cur, rand)
    if rand == goal:
        print "min_dist_to_goal: ", min_dist
    return min_dist_node

#returns the euclidean distance between two configuration
#input: two nodes

def dist(a, b):
    ans = 0
    for i in range(6):
        if not i == 4:
            ans = ans + (a.q[i] - b.q[i]) ** 2
    ans = ans + angle_diff(a.q[4], b.q[4])**2
    return sqrt(ans)


#workspace distance metric
#input, two nodes
def dist_w(a, b):
    T_a = GetEETransform(a.robot, a.q)
    T_b = GetEETransform(a.robot, b.q)
    v = T_a[0:3, 3:4] - T_b[0:3, 3:4]
    return linalg.norm(v)


#a and b are angles in radians within [-pi, pi], angle_diff returns the angle difference within [-pi, pi]
def angle_diff(a, b):
    diff = a - b
    while(diff < -pi):
        diff = diff + 2*pi
    while(diff > pi ):
        diff = diff - 2*pi
    return diff

#extend step each time towards rand until goal reached or encounter obstacle
#if goal reached, return the last node
#else, return false
def connect(near, rand, goal, step, env, robot, handles):
    cur = near
    while(True):
        new = extend(cur, rand, step)
        robot.SetActiveDOFValues(new.q)
        if env.CheckCollision(robot):
            #print "connect stopped, new node in collision"
            return False
        elif dist(new, goal) < step:
            cur.add_child(new)
            print "goal reached!"
            return new
        elif dist(new, rand) < step:
            cur.add_child(new)
            #print "connect stopped, reached random q"
            return False
        else:
            cur.add_child(new)
            #print "new node extended, q:", new.q
            cur = new

# extends step from near to rand, generate node new(initialize config, parent)
def extend(cur, rand, step):
    new_q = cur.q + step * dir(cur, rand)
    new = node(new_q, cur)
    return new

#return the unit vector pointing from cur node to rand node
def dir(cur, rand):
    diff = rand.q - cur.q
    diff[4] = angle_diff(rand.q[4], cur.q[4])
    diff = diff/linalg.norm(diff)
    return diff
#
def path(root, cur):
    path = []
    while not cur.parent == root:
        ar = ndarray.tolist(cur.q)
        path.append(ar)
        cur = cur.parent
    path.append(ndarray.tolist(root.q))
    path.reverse()
    return path

def rrt(start_config, goal_config, bias, step, n, env, robot, handles):
    node.handles = handles
    node.robot = robot
    node.env = env
    root = node(array(start_config))
    goal = node(array(goal_config))
    for k in range(n):
        rand = random_config (env, robot, goal, bias)
        near = find_near(root, rand, goal)
        last = connect(near, rand, goal, step, env, robot, handles)
        if not last == False:
            return path(root, last)


