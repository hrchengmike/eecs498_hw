import numpy as np
import matplotlib.pyplot as plt
from sgd import sgd
import time
from SGDtest import fsum, fsumprime, fsumprimeprime, fi, fiprime, fiprimeprime

def main():
    start = time.clock()
    k_s=1000
    x_vals_s = sgd (10000, fsum, fsumprime, fi, fiprime, -5, 1, k_s)
    end = time.clock()
    print "Time-sgd: ", end - start

    #convert list to numpy array and evaluate function value
    x_vals_a=np.asarray(x_vals_s)
    f_vals_s = fsum(x_vals_a)

    # plot showing f(xi) for each iteration of both methods
    plt.plot(range(k_s), f_vals_s, 'r-')
    plt.xlabel('Iteration')
    plt.ylabel('f(xi)')
    plt.xticks(np.arange(0,k_s,100))
    plt.title('''f(xi) vs i of Stochastic Gradient Descent''')
    plt.show()
    data_750 = []
    data_1000 = []
    for i in range(30):
        x_vals_s = sgd (10000, fsum, fsumprime, fi, fiprime, -5, 1, 1000)
        data_1000.append(fsum(x_vals_s[-1]))

    for i in range(30):
        x_vals_s = sgd (10000, fsum, fsumprime, fi, fiprime, -5, 1, 750)
        data_750.append(fsum(x_vals_s[-1]))

    print np.mean(data_750), np.mean(data_1000), np.var(data_750), np.var(data_1000)

if __name__ ==  '__main__':
    main()
