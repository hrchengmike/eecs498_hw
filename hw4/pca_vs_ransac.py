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
    for i in range(0,num_tests):
        pc = add_some_outliers(pc,10) #adding 10 new outliers for each test
        fig = utils.view_pc([pc])

        ###YOUR CODE HERE###
        iteration = 200
        delta = 0.2
        N = 180
        time, error = ransac(pc, iteration, delta, N)
        print "ransac: ", time, error
        time, error = pca_plane_fit(pc, delta, 0.01)
        print "pca: ", time, error
        #this code is just for viewing, you can remove or change it
        #raw_input("Press enter for next test:")
        matplotlib.pyplot.close(fig)
        ###YOUR CODE HERE###

    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()