#!/usr/bin/env python
import utils
import numpy
import matplotlib.pyplot as plt
###YOUR IMPORTS HERE###
import matplotlib
###YOUR IMPORTS HERE###

#find q in Q with min euclidean distance to p
def find_min(p, Q):
    min_dist = 10000
    for q in Q:
        dist = numpy.linalg.norm(p - q)
        if dist < min_dist:
            min_dist = dist
            min_q = q
    return min_q

# get the rigid transform from P to Q
# P and Q are lists of 3x1 numpy matrix of corresponding points
def get_transform(P, Q):
    p = utils.convert_pc_to_matrix(P)
    q = utils.convert_pc_to_matrix(Q)
    p_mean = numpy.mean(p, 1)
    q_mean = numpy.mean(q, 1)
    p = p - p_mean
    q = q - q_mean
    S = p * q.T
    u,s,v = numpy.linalg.svd(S)
    v = v.T  #Computes svd of covariance matrix Q=u*s*v'
         #output of v from numpy.linalg.svd is actualy V'
         #from Q = U S V'
    d = numpy.diag([1, 1, numpy.linalg.det(v*u.T)])
    R = v * d * u.T
    t = q_mean - R * p_mean
    return R,t

def error(source, target):
    e = 0
    for i in range(len(source)):
        e = e + numpy.linalg.norm(source[i] - target[i])**2
    return e

def main():
    #Import the cloud
    pc_source = utils.load_pc('cloud_icp_source.csv')
    ###YOUR CODE HERE###
    test = 3 # which pc_target to use, 0, 1, 2, 3
    pc_target = utils.load_pc('cloud_icp_target{}.csv'.format(str(test)))
     # Change this to load in a different target
    fig = utils.view_pc([pc_source, pc_target], None, ['b', 'r'], ['o', '^'])
    plt.axis([-0.15, 0.15, -0.15, 0.15])
    plt.show()
    error_threshold = 1.44  #0.0605 for target 0, 0.002 for target 1, 0.05 for target 2
             # 1.45 for target 3 not working
    err = [] #list of all errors
    i = 0
    while True:
        i = i+1
        print i
        Cp = []
        Cq = []
        #list of correspondence points
        #Assign correspondence points
        for p in pc_source:
            q = find_min(p, pc_target)
            Cp.append(p)
            Cq.append(q)
        #compute rigid transform
        R, t = get_transform(Cp, Cq)
        #apply transform
        for id, p in enumerate(pc_source):
            pc_source[id] = R * p + t
        cur_err = error(pc_source, pc_target)
        print "error: ", cur_err
        err.append(cur_err)
        if error(pc_source, pc_target) < error_threshold:
            break

    utils.view_pc([pc_source, pc_target], None, ['b', 'r'], ['o', '^'])
    plt.axis([-0.15, 0.15, -0.15, 0.15])

    plt.figure()
    title_font = 16
    label_font = 14
    plt.plot(range(1,len(err)+1),err, 'bo-')
    plt.title('ICP Error vs iteration (target{})'.format(test), fontsize = title_font)
    plt.xlabel('Iteration', fontsize = label_font)
    plt.ylabel('error', fontsize = label_font)
    plt.xticks(range(1,len(err)+1,3))
    plt.xticks(fontsize=label_font)
    plt.yticks(fontsize=label_font)
    plt.show()

    ###YOUR CODE HERE###

    raw_input("Press enter to end:")


if __name__ == '__main__':
    main()
