#!/usr/bin/env python
import utils
import numpy
###YOUR IMPORTS HERE###
import matplotlib.pyplot as plt
import random
import time
###YOUR IMPORTS HERE###

#fits 3-d points to a plane ax + by + cz = 1 (assume plane not passing origin)
#REQUIRES: a list of 3 x 1 numpy matrices of point coordinates (at least 3 points)
#EFFECTS: 3 x 1 ndarray of parameter [a, b, c], which is the normal vector of plane
def fit_plane(pts):
    m = len(pts)
    A = numpy.zeros((m, 3))
    B = numpy.ones((m, 1))
    for id, pt in enumerate(pts):
        A[id, :] = pt.T
    if(m < 3):
        print "Please input at least three points for fitting!"
        x = []
    elif(m == 3):
        x = numpy.linalg.inv(A).dot(B)
    else:
        x = numpy.linalg.inv(A.T.dot(A)).dot(A.T).dot(B)
    return x

#error returns the distance from the point to the plane (1 point)
# or the mean deistance from the points set to the plane
#REQUIRES:
#a list of 3 x 1 numpy matrices of point coordinates (at least 1 point),
#3 x 1 numpy array of plane parameter
#EFFECTS: returns mean distance to the plane
def error(pts, a):
    n = len(pts)
    e_tot = 0
    # compute a point x0 on the plane assuming x = 0, y = 0
    x0 = numpy.array([[0],[0],[1/a[2,0]]])
    for pt in pts:
        e = abs((pt - x0).T.dot(a)/numpy.linalg.norm(a))
        e_tot = e_tot + e
    return e_tot/n

# compute Sum of squares of residucal error
def error_rss(pts, a):
    n = len(pts)
    e_tot = 0
    for pt in pts:
        e = ((1 - a[0,0]*pt[0,0] - a[1,0]*pt[1,0])/a[2,0]-pt[2,0])**2
        e_tot = e_tot + e
    return e_tot

def ransac(pc, iteration = 50, delta = 0.15, N = 200):
    e_best = 10000
    start = time.clock()
    l = len(pc)
    for i in range(iteration):
        r = [] #r is the set of hypothetical inliers
        c = [] #c is the set of consensus set
        c_ids = []
        r_ids = []
        # randomly sample three points from point cloud
        r_ids = random.sample(range(0, l), 3)
        for r_id in r_ids:
            r.append(pc[r_id])
        # fit plane to hypothetical inliers
        plane = fit_plane(r)
        # generate c-consensus set
        for id, pt in enumerate(pc):
            if (id not in r_ids) and error([pc[id]], plane) < delta:
                c.append(pc[id])
                c_ids.append(id)
        # refit model if consensus set is sufficiently big
        if len(c) > N:
            plane_refit = fit_plane(r+c)
            e_new = error_rss(r+c, plane_refit)
            if e_new < e_best:
                e_best = e_new
                plane_best = plane_refit
                r_ids_best = r_ids
                c_ids_best = c_ids
    end = time.clock()

    #Show the resulting point cloud
    outliers = [] #plot outliers
    inliers = []#inliers
    for id, pt in enumerate(pc):
        if id not in r_ids_best+c_ids_best:
            outliers.append(pt)
        else:
            inliers.append(pt)
    fig = utils.view_pc([outliers])

    #plot inliers
    utils.view_pc([inliers], fig, 'r', 'o')

    #Draw the fitted plane
    pt = numpy.matrix([[0],[0],[1/plane_best[2,0]]])
    utils.draw_plane(fig, plane_best, pt, (0.1, 0.7, 0.1, 0.5), length=[-0.5, 1], width=[-0.5, 1])
    plt.show()
    plt.title("Plane fitting result of RANSAC algorithm", fontsize = 16)
    return end-start, e_best

def main():
    #Import the cloud
    pc = utils.load_pc('cloud_ransac.csv')

    ###YOUR CODE HERE###
    # Show the input point cloud
    #fig = utils.view_pc([pc])

    #Fit a plane to the data using ransac
    iteration = 50
    delta = 0.15
    N = 200
    time, error = ransac(pc, iteration, delta, N)
    print time, error
    ###YOUR CODE HERE###
    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
