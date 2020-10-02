#!/usr/bin/env python
from backtracking import backtracking

### newton's method of single variable function
# EFFECTS: outputs x_vals and f_vals as list of x and f values for
# each iteration and k, the number of iteration
# REQUIRES: backtracking()
# f : objective function
# x0 : starting value
# h : step size of Symmetric Difference Quotient
# e : tolerance
# alpha, beta: input parameter for backtracking line search
def newtonsmethod(f, x0, h, e, alpha, beta):
    x = x0
    k = 0 #number of iteration
    #initialize list that stores results of each iteration
    x_vals = []
    f_vals = []
    while 1:
        # store each x, f(x) into list
        x_vals.append(x)
        f_vals.append(f(x))

        df = (f(x + h) - f(x - h)) / (2 * h) #first derivative
        ddf = (f(x + h) - 2*f(x) + f(x - h)) / (h * h) #second derivative
        dx = -df / ddf
        lambda_square = df * df / ddf
        if (lambda_square / 2 <= e):
            return x_vals, f_vals, k
        t = backtracking(f, x, alpha, beta, h)
        x = x + t*dx
        k = k + 1
