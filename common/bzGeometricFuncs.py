#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Jul 23 14:22:58 2018

@author: bzorn
"""

import numpy as np
import shapely.geometry as geo


#Projects a point (usually baselink coordinates) to a trajectory and returns
#the distance to that trajectory as well as the index of the next point on the trajectory
#that should be used as the desired heading
#def BaseLinkToTrajectory(baselink_x, baselink_y, trajectory_xvals, trajectory_yvals):
def BaseLinkToTrajectory(baselink_x, baselink_y, trajectory_poses):
    
    [trajectory_xvals, trajectory_yvals] = PathToValues(trajectory_poses)
    
    geoBaselink = geo.Point(baselink_x, baselink_y)
    trajectory = np.array([trajectory_xvals, trajectory_yvals])
    trajectory = np.transpose(trajectory)
    geoTrajectory = geo.LineString(trajectory)
    f_dist = geoBaselink.distance(geoTrajectory) #distance between trajectory and baselink origin
    f_projLength = geoTrajectory.project(geoBaselink, normalized=True) #projection of Baselink to closest point on trajectory with distance f_dist
    
    b_DirectionFound = False
    nDirection = 0
    idx = int(round(len(geoTrajectory.coords)*f_projLength))
    while(True):
#        if idx == 0: #is starting point
#            idx += 1
        if idx == len(geoTrajectory.coords):
            idx -= 1
        else:
            if(b_DirectionFound == False):
                print("x")
                f_projLengthPointBefore = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx]), normalized=True)
                print("y")
                f_projLengthPointAfter = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx+1]), normalized=True)
            elif(nDirection == 1):
                f_projLengthPointAfter = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx+1]), normalized=True)
            else:
                f_projLengthPointBefore = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx]), normalized=True)
                
            if (f_projLengthPointBefore <= f_projLength and f_projLengthPointAfter >= f_projLength):
                idx += 1
                break;
            else:
                if (nDirection == 0):
                    if(f_projLengthPointAfter-f_projLength < f_projLengthPointBefore-f_projLength):
                        nDirection = 1
                    else:
                        nDirection = -1
                    b_DirectionFound = True
            if(nDirection == 1):
                idx += 1
                f_projLengthPointBefore = f_projLengthPointAfter
            elif(nDirection == -1):
                idx -= 1
                f_projLengthPointAfter = f_projLengthPointBefore


    return [f_dist, idx]


def PathToValues(trajectory_poses):
    xvals = []
    yvals = []
    for pose in trajectory_poses:
        xvals.append(pose.pose.position.x)
        yvals.append(pose.pose.position.y)
    return [xvals, yvals]
    # return [f_dist, f_heading]

#    