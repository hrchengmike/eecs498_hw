#!/usr/bin/env python
from backtracking import backtracking

### newton's method of single variable function
# EFFECTS: outputs x_vals ]as list of x for each iteration and k, the number of iteration
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
    while 1:
        #print k, x, f(x)
        # store each x into list
        x_vals.append(x)

        df_val=df(x)
        ddf_val=ddf(x)
        dx = -df_val / ddf_val
        lambda_square = df_val * df_val / ddf_val
        if (lambda_square / 2 <= e):
            return x_vals,  k
        t = backtracking(f, df, x, dx, alpha, beta)
        x = x + t*dx
        k = k + 1
