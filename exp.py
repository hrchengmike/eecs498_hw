from bisect import bisect_left
print "hello world"
from numpy import *
# some_file.py
import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, 'hw3')
from rrt import *

class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid
    '''
    def __eq__(self, other):
        round(self.x, 3) == round(other.x, 3)
        round(self.y, 3) == round(other.y, 3)
    '''
    def __lt__(self, other):
        return self.id < other.id

def index(a, x):
    'Locate the leftmost value exactly equal to x'
    i = bisect_left(a, x)
    if i != len(a) and a[i] == x:
        return i
    raise ValueError

def angle_diff(a, b):
    diff = a - b
    while(diff < -pi):
        diff = diff + 2*pi
    while(diff > pi ):
        diff = diff - 2*pi
    return diff

if __name__ == "__main__":
    dict = {(1.22, 3.222):1, (round(3.23424141, 5), 3.22):1}
    print type(1.22)
    print dict.has_key((3.222, 3.22))
    print dict.has_key((round(3.23424142, 5),3.22))
    ls = [0, 2, 4, 5, 6 ,8, 9]
    print index(ls, 6)
    ls_node = []
    n1 = Node(0,0,0,0,0)
    n2 = Node(1,0,0,1,0)
    n3 = Node(0,1,0,2,0)
    n4 = Node(2,0,0,3,1)
    ls_node = [n1, n2, n3, n4]
    print bisect_left(ls_node, Node(0, 0, 0, 4, 0))
    '''
    list = array([])
    list.append(array([3]))
    list.append(array([4]))
    list.append(array([3,4]))
    print list
    print list[0]
    print list[2]
    '''
    print round(3.23)
    c = arange(0,1,0.1)
    for a in c:
        print (a)
    print "starting diff"
    a = angle_diff(pi, -3)
    print a
    root = node(array([1, 2, 3, 2, pi/2,  2]))
    n1 = node(array([3, 4, 5, 4, -pi/2-0.00001, 1]))
    print dir(root, n1)
    a = array([1, 1, 1, 1, 1, 1])
    print a*2 + dir(root, n1)
    a = array([[1,2],[3,4]])
    b = a + 4
    c = b + 4
    d = c + 4
    print random.rand()
    print vstack([array([]), hstack([c,d])])

