import numpy as np
import matplotlib.pyplot as plt
import time
import random
from sgd import sgd
from gradientdescent import gradientdescent
from newtonsmethod import newtonsmethod

maxi = 10000 #this is the number of functions

def fi(x,i):
    coef1 = 0.01 + (0.5-0.01)*i/maxi
    coef2 = 1 + (6-1)*i/maxi
    return (np.exp(coef1*x + 0.1) + np.exp(-coef1*x - 0.5) - coef2*x)/(maxi/100)

def fiprime(x,i):
    coef1 = 0.01 + (0.5-0.01)*i/maxi
    coef2 = 1 + (6-1)*i/maxi
    return (coef1*np.exp(coef1*x + 0.1) -coef1*np.exp(-coef1*x - 0.5) - coef2)/(maxi/100)

def fiprimeprime(x,i):
    coef1 = 0.01 + (0.5-0.01)*i/maxi
    #coef2 = 1 + (6-1)*i/maxi
    return (coef1*coef1*np.exp(coef1*x + 0.1) +coef1*coef1*np.exp(-coef1*x - 0.5))/(maxi/100)


def fsum(x):
    sum = 0
    for i in range(0,maxi):
       sum = sum + fi(x,i)
    return sum

def fsumprime(x):
    sum = 0
    for i in range(0,maxi):
       sum = sum + fiprime(x,i)
    return sum

def fsumprimeprime(x):
    sum = 0
    for i in range(0,maxi):
       sum = sum + fiprimeprime(x,i)
    return sum

def main():
    start = time.clock()
    x_vals_s = sgd (10000, fsum, fsumprime, fi, fiprime, -5, 1, 1000)
    end = time.clock()
    print "Time-sgd: ", end - start

    start = time.clock()
    [x_vals_g, k_g] = gradientdescent(fsum, fsumprime, -5, 0.0001, 0.1, 0.6)
    end = time.clock()
    print "Time-gradient descent: ", end - start

    start = time.clock()
    [x_vals_n, k_n] = newtonsmethod(fsum, fsumprime, fsumprimeprime, -5, 0.0001, 0.1, 0.6)
    end = time.clock()
    print "Time-newton's method: ", end - start

    print 'SGD fsum(x*): ', fsum(x_vals_s[-1])
    print 'Gradient Descent fsum(x*): ', fsum(x_vals_g[-1])
    print '''Newton's method fsum(x*): ''', fsum(x_vals_n[-1])

if __name__ ==  '__main__':
    main()

