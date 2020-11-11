#!/usr/bin/env python
import utils
import numpy
import time
import random
import matplotlib
###YOUR IMPORTS HERE###
from ransac_template import *
from pca_template import *
###YOUR IMPORTS HERE###

def add_some_outliers(pc,num_outliers):
    pc = utils.add_outliers_centroid(pc, num_outliers, 0.75, 'uniform')
    random.shuffle(pc)
    return pc

def main():
    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')

    num_tests = 10
    fig = None
    err_ransac = []
    time_ransac = []
    err_pca = []
    time_pca = []
    for i in range(0,num_tests):
        pc = add_some_outliers(pc,10) #adding 10 new outliers for each test
        fig = utils.view_pc([pc])

        ###YOUR CODE HERE###
        iteration = 500
        delta = 0.2
        N = 180
        time, error = ransac(pc, iteration, delta, N)
        err_ransac.append(error)
        time_ransac.append(time)
        time, error = pca_plane_fit(pc, delta)
        err_pca.append(error)
        time_pca.append(time)
        #this code is just for viewing, you can remove or change it
        #raw_input("Press enter for next test:")
        matplotlib.pyplot.close(fig)
    plt.figure()
    plt.plot(range(10,10*num_tests+1,10),err_ransac,'bo-')
    plt.plot(range(10,10*num_tests+1,10),err_pca,'ro-')
    plt.title("Error - number of outliers")
    plt.xlabel("number of outliers")
    plt.ylabel("residual sum of squares")
    plt.legend(["RANSAC","PCA"])
    print "RANSAC time, mean: ", numpy.mean(time_ransac), "variance: ", numpy.var(time_ransac)
    print "PCA time, mean: ", numpy.mean(time_pca), "variance: ", numpy.var(time_pca)
        ###YOUR CODE HERE###

    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
