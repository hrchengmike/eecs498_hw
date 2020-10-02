#!/usr/bin/env python
from random import randrange


### stochastic gradient descent algorithm of single variable function
# EFFECTS: outputs x_vals as list of x  values for
# each iteration and k, the number of iteration
# i : number of functions
# fsum : objective function
# fsumprime: first derivative of fsum
# fi: each function component
# fiprime: first derivative of fi
# x0 : starting value
# t: fixed step size
# max_iteration: number of iteration
def sgd(i, fsum, fsumprime, fi, fiprime, x0, t, max_iteration):
    x_vals = []
    x = x0
    k = 0 #number of iteration
    while k < max_iteration:
        # store each x, f(x) into separate list
        x_vals.append(x)

        i_rand = randrange(0, i+1)
        dx = - fiprime(x, i_rand)
        x = x + t*dx
        k = k + 1

    return x_vals

