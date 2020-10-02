#!/usr/bin/env python

#EECS HW1 Python code
#Mike Haoran Cheng
#9-21-2020
import math
import numpy as np
from numpy import cos as cos
from numpy import sin as sin

def main():
    #Matrices-Problem 5 a
    print 'Problem 5 (a)'
    A = np.matrix([[0, 0, -1], [4, 1, 1], [-2, 2, 1]])
    b = np.matrix([[3], [1], [1]])
    print 'A= \n', A
    print 'b= \n', b
    print 'rank(A) \n', np.linalg.matrix_rank(A)
    if np.linalg.matrix_rank(A)== len(A):
        x = np.linalg.solve(A, b)
        print 'Exact solution x= \n', x
    else:
        x = np.linalg.lstsq(A,b)[0]
        print 'Least square soltion x= \n', x

    #Matrices-Problem 5 b
    print 'Problem 5 (b)'
    A = np.matrix([[0, -2, 6],[-4, -2, -2],[2, 1, 1]])
    b = np.matrix([[1], [-2], [0]])
    print 'A= \n', A
    print 'b= \n', b
    print 'rank(A) \n', np.linalg.matrix_rank(A)
    if np.linalg.matrix_rank(A) == len(A):
        x = np.linalg.solve(A, b)
        print 'Exact solution x= \n', x
    else:
        x = np.linalg.lstsq(A,b)[0]
        print 'Least square soltion x= \n', x

    #Matrices-Problem 5 c
    print '\nProblem 5 (c)'
    A = np.matrix([[2, -2],[-4, 3]])
    b = np.matrix([[3], [-2]])
    print 'A= \n', A
    print 'b= \n', b
    print 'rank(A) \n', np.linalg.matrix_rank(A)
    if np.linalg.matrix_rank(A) == len(A):
        x = np.linalg.solve(A, b)
        print 'Exact solution x= \n', x
    else:
        x = np.linalg.lstsq(A, b)[0]
        print 'Least square soltion x= \n', x

    # Matrices - Problem 6
    print '\nProblem 6'
    A = np.matrix([[3, 2],[1, -2]])
    B = np.matrix([[-1, -2],[4, -2]])
    print 'A+2B= \n', A+2*B
    print 'AB= \n', A*B
    print 'BA= \n', B*A
    print 'A transpose= \n', np.transpose(A)
    print 'B^2= \n', B**2
    print 'AtBt= \n ', np.transpose(A)*np.transpose(B)
    print '(AB)t= \n', np.transpose(A*B)
    print 'det(A)= \n', np.linalg.det(A)
    print 'inverse of B= \n', np.linalg.inv(B)

    #Rotation- Problem 7
    print '\nProblem 7'
    def rotx(theta):
        M = np.matrix([[1, 0, 0] , [0, cos(theta), -sin(theta)] , [0, sin(theta) , cos(theta)]])  # noqa: E203
        return M
    def roty(theta):
        M = np.matrix([[cos(theta), 0, sin(theta)],[0, 1, 0],[-sin(theta), 0, cos(theta)]])
        return M
    def rotz(theta):
        M = np.matrix([[cos(theta), -sin(theta), 0],[sin(theta), cos(theta), 0],[0, 0, 1]])
        return M
    ans = rotz(math.pi/3)*roty(math.pi/4)*rotz(math.pi)
    print 'ans= \n', ans

    #Rotation- Problem 8 a,b
    print '\nProblem 8'
    p0r = np.matrix([[1.7],[2.1],[0]])
    R0r = rotz(-math.pi/4)
    T0r = np.block([[R0r, p0r],[0,0,0,1]])
    #print T0r
    p=np.matrix([[0.1],[-1.4],[0]])
    v = (p-p0r)/np.linalg.norm(p-p0r)
    print 'vn is \n', v
    ry = rotz(math.pi/2)*v
    print 'ry = \n', ry
    rz = np.matrix([[0], [0], [1]])
    R = np.block([v, ry, rz])
    print 'R = \n', R
    T_desired = np.block([[R, p0r], [0, 0, 0, 1]])
    print 'T_desired = \n', T_desired

    #Problem 8 c
    print R*R.transpose()
    print np.linalg.det(R)

if __name__ == '__main__':
    main()
