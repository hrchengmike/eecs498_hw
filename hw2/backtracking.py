#!/usr/bin/env python

### backtracking line search for single varible function
# f : objective function
# x : evaluate at x
# alpha, beta: input parameter for backtracking line search
# df(x): first derivative of f
def backtracking(f, df, x, dx, alpha, beta):
    t = 1
    while f(x + t*dx) > f(x) + alpha * t * (df(x) * dx):
        t = beta * t
    return t
