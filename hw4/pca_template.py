#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###
import matplotlib.pyplot as plt
from ransac_template import error, error_rss
import time
###YOUR IMPORTS HERE###

#drops the dimension with lowest singular value
def pca_plane_fit(pc, delta):
    start = time.clock()
    #Rotate the points to align with the XY plane
    n = len(pc) #number of data in the point cloud
    x_raw = utils.convert_pc_to_matrix(pc)
    pt = numpy.mean(x_raw, 1)
    x = x_raw - pt
    Q = x*x.T/(n-1)
    u,s,v = numpy.linalg.svd(Q)
    v = v.T  #Computes svd of covariance matrix Q=u*s*v'
             #output of v from numpy.linalg.svd is actualy V'
             #from Q = U S V'
    # fit plane to original pc

    variances = numpy.multiply(s,s)
    normal = v[:, 2]
    plane = normal/(normal.T.dot(pt))

    #generate inliers and outliers
    inliers = []
    outliers = []
    for point in pc:
        if error([point], plane) < delta:
            inliers.append(point)
        else:
            outliers.append(point)
    rss_err = error_rss(inliers, plane)
    end = time.clock()

    #plot
    fig = utils.view_pc([outliers])
    utils.view_pc([inliers], fig, 'r', 'o')
    pt = numpy.matrix([[0],[0],[1/plane[2,0]]])
    utils.draw_plane(fig, plane, pt, (0.1, 0.7, 0.1, 0.5), length=[-0.5, 1], width=[-0.5, 1])
    plt.title("Plane fitting result of PCA", fontsize = 16)
    #Draw the fitted plane
    plt.show()
    return end-start, rss_err
def main():

    #Import the cloud
    pc = utils.load_pc('cloud_pca.csv')


    ###YOUR CODE HERE###
    # Show the input point cloud
    utils.view_pc([pc])

    #Rotate the points to align with the XY plane
    n = len(pc) #number of data in the point cloud
    x = utils.convert_pc_to_matrix(pc)
    x = x - numpy.mean(x, 1)
    Q = x*x.T/(n-1)
    u,s,v = numpy.linalg.svd(Q)
    v = v.T  #Computes svd of covariance matrix Q=u*s*v'
             #output of v from numpy.linalg.svd is actualy V'
             #from Q = U S V'
    y = v.T*x
    pc_y = utils.convert_matrix_to_pc(y)

    #Show the resulting point cloud
    fig = utils.view_pc([pc_y])
    #plt.gca().set_aspect('equal', adjustable='box')

    #Rotate the points to align with the XY plane AND eliminate the noise
    v_s = v.copy()
    variances = numpy.multiply(s,s)
    e = 0.001 # threshold of variance, below which that dimension will be eliminated
    for id, variance in enumerate(variances):
        if variance < e:
            v_s[:,id] = numpy.zeros((v.shape[0],1))
    y_s = v_s.T*x
    pc_ys = utils.convert_matrix_to_pc(y_s)

    # Show the resulting point cloud
    utils.view_pc([pc_ys])

    # fit plane to original pc
    x = utils.convert_pc_to_matrix(pc)
    pt = numpy.mean(x, 1)
    normal = v[:, variances < e]
    fig = utils.view_pc([pc])
    utils.draw_plane(fig, normal, pt, (0.1, 0.7, 0.1, 0.5), length=[-0.5, 1], width=[-0.5, 1])
    ###YOUR CODE HERE###


    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
