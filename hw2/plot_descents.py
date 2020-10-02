#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
from backtracking import backtracking
from gradientdescent import gradientdescent
from newtonsmethod import newtonsmethod

# f(x) is the object function of problem 1
def f(x):
    return np.exp(0.5 * x + 1) + np.exp(-0.5 * x - 0.5) + 5 * x

def main():
    [x_vals_g, f_vals_g, k_g]= gradientdescent(f, 5, 0.00001, 0.0001, 0.1, 0.6)
    [x_vals_n, f_vals_n, k_n] = newtonsmethod(f, 5, 0.00001, 0.0001, 0.1, 0.6)

    #plot the objective function and sequence of points
    plt.figure(1)
    xvals = np.arange(-10, 10, 0.01) # Grid of 0.01 spacing from -10 to 10
    yvals = f(xvals) # Evaluate function on xvals
    plt.plot(xvals, yvals, 'k')
    plt.plot(x_vals_g, f_vals_g, 'rx')
    plt.plot(x_vals_n, f_vals_n, 'mx')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('''Gradient descent and Newton's method: sequence of points''')
    plt.legend({'objective function','gradient descent','''newton's method'''})
    plt.show() #show the plot

    # plot showing f(xi) for each iteration of both methods
    plt.figure(2)
    plt.plot(range(k_g+1), f_vals_g, 'rx')
    plt.plot(range(k_n+1), f_vals_n, 'mx')
    plt.xlabel('Iteration')
    plt.ylabel('f(xi)')
    plt.legend(['gradient descent','''newton's method'''])
    plt.xticks(np.arange(k_g+1))
    plt.title('''f(xi) vs i of Gradient descent and Newton's method''')
    plt.show()


if __name__ ==  '__main__':
    main()

