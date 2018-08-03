#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  9 13:09:55 2018

@author: bzorn
"""

import matplotlib.pyplot as plt
import scipy.interpolate
import numpy as np

def vangle(v1, v2):
    retval = np.arccos(np.clip(np.dot(v1, v2), -1, 1)) 
    print(retval)
    return retval #clips numerical imprecision to -1 and 1 in order to adhere to arccos bounds

def vangle2(v1, v2):
    dot = v1[0]*v2[0]+v1[1]*v2[1] #np.dot
    det = v1[0]*v2[1]-v1[1]*v2[0] #np.linalg.det
    angle = np.arctan2(det, dot)
    return angle


#Width should be set according to expected track width
def EgoTrajectories(f_lateralDist, f_longitudinalDist, n_amount, b_verbose):

    xnew = np.linspace(0, f_longitudinalDist, num=41)

    Traj_Cubic = []
    Traj_Quad = []
    
    
    for s1 in np.linspace(-float(f_lateralDist)/2, float(f_lateralDist)/2, n_amount):
        x = [0, f_longitudinalDist]
        y = [0, s1]
        f = scipy.interpolate.CubicSpline(x,y, bc_type='clamped')
        ynew1 = [f(x) if abs(f(x))<= 1 else np.sign(f(x))*1 for x in xnew]
#        plt.plot(f(xnew), xnew)
        Traj_Cubic.append(f(xnew))

    for s1 in np.linspace(-float(f_lateralDist)/2, float(f_lateralDist)/2, n_amount):
        x = [0, float(f_longitudinalDist)/2+float(f_longitudinalDist)/10, f_longitudinalDist]
        y = [0, s1/2, s1]
        f = scipy.interpolate.interp1d(x, y, kind='quadratic')
        ynew2 = [f(x) if abs(f(x))<= 1 else np.sign(f(x))*1 for x in xnew]
#        plt.plot(f(xnew), xnew)
        Traj_Quad.append(f(xnew))
    
    if b_verbose is True:
    
#        [plt.plot(trajectory, xnew, c='r', alpha=0.8) for trajectory in Traj_Cubic]
#
#        plt.plot(Traj_Cubic[0], xnew, c='r', alpha=0.8, label = 'cubic clamped')
#        [plt.plot(trajectory, xnew, c='b', alpha=0.5) for trajectory in Traj_Quad]
#        plt.plot(Traj_Quad[0], xnew, c='b', alpha=0.8, label = 'quadratic')
#        plt.plot(len(xnew)*[0], xnew, c='g', label='linear')
#        plt.legend(loc='best')
#        plt.title('EgoTrajectories - All')
#        plt.xlabel('[m]')
#        plt.ylabel('[m]')
#        plt.show()

        n_counter = 1
        plt.plot(len(xnew)*[0], xnew, c='g', label='0')
        for trajectory in Traj_Cubic:
            plt.plot(trajectory, xnew, label=str(n_counter), linestyle='--')
            n_counter += 1
        for trajectory in Traj_Quad:
            plt.plot(trajectory, xnew, label=str(n_counter))
            n_counter += 1

        plt.legend(loc='best')
        plt.title('EgoTrajectories - All')
        plt.xlabel('[m]')
        plt.ylabel('[m]')
        plt.gca().invert_xaxis()
        plt.show()


#    debug = np.ndarray((1, len(xnew)))
#    a = np.linspace(0, 7.15, len(xnew))
#    debug[0] = a
#    return [xnew,  np.concatenate((debug, Traj_Cubic, Traj_Quad), axis=0)] #first parameter is default option

    return [xnew, np.concatenate((np.zeros((1,len(xnew))), Traj_Cubic, Traj_Quad), axis=0)] #first parameter is default option



    
def ComputeHeadingsInRad(xvals, yvals):
    '''
    Compute orientation from vectors between two subsequent waypoints
    
    Arguments:
    xvals -- euclidean coordinates X
    yvals -- euclidean coordinates Y
    
    Returns:
    headings -- list of orientation angles in radians
    '''
    headings = []
    y_axis = np.array([0, -1])
    for n in range(0, len(xvals)-1): #manually set last heading to same as before
        v1 = np.array([xvals[n], yvals[n]])
        v2 = np.array([xvals[n+1], yvals[n+1]])
        headings.append(vangle2(v2-v1, y_axis))
    headings.append(headings[-1])
#    print(len(xvals), len(yvals))
    return headings

#EgoTrajectories()
#CompareSecondOrderPolynomials()
#CompareThirdOrderPolynomials()
#
#f_lateralDist, f_longitudinalDist, n_amount, b_verbose = 10, 25, 6, True
#EgoTrajectories(f_lateralDist, f_longitudinalDist, n_amount, b_verbose)