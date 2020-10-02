#!/usr/bin/env python
from backtracking import backtracking

### gradient descent algorithm of single variable function
# EFFECTS: outputs x_vals as list of x values for
# each iteration and k, the number of iteration
# REQUIRES: backtracking()
# f : objective function
# df(x): first derivative of f
# x0 : starting value
# e : tolerance
# alpha, beta: input parameter for backtracking line search

def gradientdescent(f, df, x0, e, alpha, beta):
    x_vals = []
    x = x0
    k = 0 #number of iteration
    while 1:
        #print k, x, f(x)
        # store each x, f(x) into separate list
        x_vals.append(x)

        dx = -df(x)
        t = backtracking(f, df, x, dx, alpha, beta)
        x = x + t*dx
        if (abs(df(x)) <= e):
            return x_vals, k
        k = k + 1

