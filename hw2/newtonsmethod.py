#!/usr/bin/env python
from backtracking import backtracking

### newton's method of single variable function
# EFFECTS: outputs x_vals and f_vals as list of x and f values for
# each iteration and k, the number of iteration
# REQUIRES: backtracking()
# f : objective function
# df(x): first derivative of f
# ddf(x): second derivative of f
# x0 : starting value
# e : tolerance
# alpha, beta: input parameter for backtracking line search
def newtonsmethod(f, df, ddf, x0, e, alpha, beta):
    x = x0
    k = 0 #number of iteration
    #initialize list that stores results of each iteration
    x_vals = []
    f_vals = []
    while 1:
        # store each x, f(x) into list
        x_vals.append(x)
        f_vals.append(f(x))

        dx = -df(x) / ddf(x)
        lambda_square = df(x) * df(x) / ddf(x)
        if (lambda_square / 2 <= e):
            return x_vals, f_vals, k
        t = backtracking(f, df, x, alpha, beta)
        x = x + t*dx
        k = k + 1
