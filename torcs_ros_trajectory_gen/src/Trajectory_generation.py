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
#
#def curve(a1, a2, x):
#    y = s0+a1*x*x+a2*x
#    if abs(y) <= 1:
#        return y
#    else:
#        return np.sign(y)*1 
#    
#def CurveTrajectories():
#    s0 = 0.8
#    
#    t1 = 3
#    t2 = 5
#    # s_out = so+ a1*t*t 
#    xnew = np.linspace(0, 5, num=41)
#    
#    
#    #Network determines goal steering, and time frame interpolation done manually (could be implemented with or without action choice)
#    for s1 in np.linspace(-1, 1, 10):
#        rand = 5
#        while (rand > (t2-t1) or rand < -t1):
#            rand = np.random.normal(loc=0, scale=2)
#        x = np.array([0, t1+rand, t2])
#        y = np.array([s0, s1, s1])
#        f2 = scipy.interpolate.interp1d(x, y, kind='quadratic')
#        ynew = [f2(x) if abs(f2(x))<= 1 else np.sign(f2(x))*1 for x in xnew]
#        plt.plot(xnew, ynew)
#    plt.title('Interp1d: Quadratic')
#    plt.show()
#        #smooth cubic interpolation
#    for s1 in np.linspace(-1, 1, 50):
#        x = [0, t2]
#        y = [s0, s1]
#        f = scipy.interpolate.CubicSpline(x,y, bc_type='clamped')
#        ynew = [f(x) if abs(f(x))<= 1 else np.sign(f(x))*1 for x in xnew]
#        plt.plot(xnew, f(xnew))
#    plt.title('CubicSpline - Clamped')
#    plt.show()
#    
#    for s1 in np.linspace(-1, 1, 50):
#        rand = 5
#        while (rand > (t2-t1) or rand < -t1):
#            rand = np.random.normal(loc=0, scale=0.8)
#        x = [0, t1+rand, t2]
#        y = [s0, s1, s1]
#        f = scipy.interpolate.CubicSpline(x,y, bc_type='clamped')
#        ynew = [f(x) if abs(f(x))<= 1 else np.sign(f(x))*1 for x in xnew]
#        plt.plot(xnew, ynew)
#    plt.title('CubicSpline - Clamped - Noisy')
#    plt.show()
#    #Network decides between set of parameters (Calculation can then be performed by a network) --> entirely possible in neuromorph
#    for n in np.linspace(-1, 1, 50):
#        a1 = np.random.uniform(-0.05, 0.05)
#        a2 = np.random.uniform(-0.05, 0.05)
#        ynew = [curve(a1, a2, x) for x in xnew]
#        plt.plot(xnew, ynew)
#    plt.title('Parametrized')
#    plt.show()

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
    if b_verbose is True:
        plt.title('EgoTrajectories - Cubic Clamped')
        plt.xlabel('[m]')
        plt.ylabel('[m]')
        plt.show()
        
    for s1 in np.linspace(-float(f_lateralDist)/2, float(f_lateralDist)/2, n_amount):
        x = [0, float(f_longitudinalDist)/2+float(f_longitudinalDist)/10, f_longitudinalDist]
        y = [0, s1/2, s1]
        f = scipy.interpolate.interp1d(x, y, kind='quadratic')
        ynew2 = [f(x) if abs(f(x))<= 1 else np.sign(f(x))*1 for x in xnew]
#        plt.plot(f(xnew), xnew)
        Traj_Quad.append(f(xnew))
    
    if b_verbose is True:
        plt.title('EgoTrajectories - Quadratic')
        plt.xlabel('[m]')
        plt.ylabel('[m]')
        plt.show()
    
        [plt.plot(trajectory, xnew, c='r', alpha=0.8) for trajectory in Traj_Cubic]
        plt.plot(Traj_Cubic[0], xnew, c='r', alpha=0.8, label = 'cubic clamped')
        [plt.plot(trajectory, xnew, c='b', alpha=0.5) for trajectory in Traj_Quad]
        plt.plot(Traj_Quad[0], xnew, c='b', alpha=0.8, label = 'quadratic')
        plt.plot(len(xnew)*[0], xnew, c='g', label='linear')
        plt.legend(loc='best')
        plt.title('EgoTrajectories - All')
        plt.xlabel('[m]')
        plt.ylabel('[m]')
        plt.show()

    return [xnew, np.concatenate((np.zeros((1,len(xnew))), Traj_Cubic, Traj_Quad), axis=0)] #first parameter is default option


#def CompareSecondOrderPolynomials():
#    a, b, c = 1,1, 1
#    x = np.linspace(0, 1, 10)
#    [plt.plot((a+n)*x*x+b*x+c, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('a*x*x')
#    plt.show()
#    [plt.plot((a)*x*x+(b+n)*x+c, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('b*n')
#    plt.show()
#    [plt.plot(a*x*x+b*x+c+n, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('c')
#    plt.show()
#
#
##    b = 5
##    [plt.plot((a+n)*x*x+b*x+c, x) for n in np.linspace(-5, 5, 20)]
##    plt.title('a*x*x')
##    plt.show()
#
## jegliches Polynom kann so rotiert werden, dass es stetig an ein anderes angeknüpft werden kann
#def CompareThirdOrderPolynomials():
#    a, b, c, d = 1, 1, 1, 1
#    x = np.linspace(0, 1, 10)
#    [plt.plot((d+n)*x*x*x+a*x*x+b*x+c, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('d*x*x*x')
#    plt.show()
#    [plt.plot(d*x*x*x+(a+n)*x*x+b*x+c, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('a*x*x')
#    plt.show()
#    [plt.plot(d*x*x*x+(a)*x*x+(b+n)*x+c, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('b*n')
#    plt.show()
#    [plt.plot(d*x*x*x+(a+n)*x*x+b*x+c+n, x) for n in np.linspace(-5, 5, 20)]
#    plt.title('c')
#    plt.show()
#    
    
def ComputeHeadingsInRad(xvals, yvals):
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
