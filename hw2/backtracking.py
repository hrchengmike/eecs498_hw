#!/usr/bin/env python

### backtracking line search for single varible function
# f : objective function
# x : evaluate at x
# alpha, beta: input parameter for backtracking line search
# h : step size of Symmetric Difference Quotient
def backtracking(f, x, alpha, beta, h):
    t = 1
    df = (f(x + h) - f(x - h)) / (2 * h)
    dx = - df
    while f(x + t*dx) > f(x) + alpha * t * (df * dx):
        t = beta * t
    return t
