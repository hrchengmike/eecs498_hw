#!/usr/bin/env python
from backtracking import backtracking

### gradient descent algorithm of single variable function
# EFFECTS: outputs x_vals and f_vals as list of x and f values for
# each iteration and k, the number of iteration
# REQUIRES: backtracking()
# f : objective function
# x0 : starting value
# h : step size of Symmetric Difference Quotient
# e : tolerance
# alpha, beta: input parameter for backtracking line search

def gradientdescent(f, x0, h, e, alpha, beta):
    x_vals = []
    f_vals = []
    x = x0
    k = 0 #number of iteration
    while 1:
        # store each x, f(x) into separate list
        x_vals.append(x)
        f_vals.append(f(x))

        df = (f(x + h) - f(x - h)) / (2 * h)
        dx = -df
        t = backtracking(f, x, alpha, beta, h)
        x = x + t*dx
        if (abs(df) <= e):
            return x_vals, f_vals, k
        k = k + 1
