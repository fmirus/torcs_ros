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
    
    [trajectory_xvals, trajectory_yvals] = PathToValues(trajectory_poses) #get list of position values from path message
    
    geoBaselink = geo.Point(baselink_x, baselink_y) #create a shapely point entity for the baselink position
    trajectory = np.array([trajectory_xvals, trajectory_yvals]) #recombine trajectory points to numpy array
    trajectory = np.transpose(trajectory) #transpose needed for shapely
    geoTrajectory = geo.LineString(trajectory) #create a shapely polyline entity for the trajectory
    f_dist = geoBaselink.distance(geoTrajectory) #distance between trajectory and baselink origin
    f_projLength = geoTrajectory.project(geoBaselink, normalized=True) #projection of Baselink to closest point on trajectory with distance f_dist
    f_distToEnd = geo.Point(geoTrajectory.coords[-1]).distance(geoBaselink) #result how far from end of trajectory current baselink is located
    
    #we are going to search for the nearest defined polyline point in order to determine the vehicle's desired heading
    #this is done by finding its bounding points (comparing two points projected length to the projected length of the point;
    #if one is higher and one lower we have identified the bounding points)
    #in order to save some time we will estimate a starting point and move along the polyline until we can identify the nearest point
    b_DirectionFound = False #indicator flag to perform a DoOnce operation
    nDirection = 0 #indicates whether search direction is towards start or end of polyline
    idx = int(round(len(geoTrajectory.coords)*f_projLength)) #currently inspected index, initialization with normalized proj_length to estimate a starting point that might be close
    idx = np.clip(idx, 0, len(geoTrajectory.coords)-1) #limit to valid range
    while(True): #repeat until nearest point is identified
        if idx >= len(geoTrajectory.coords)-1: #consider boundary case where last point was assumed as starting point
            idx -= 1 #consider penultimate point instead //was just -1
        else:
            if(b_DirectionFound == False): #see if this is the first loop
                try:
                    f_projLengthPointBefore = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx]), normalized=True) #get the lenght along the line of idx
                except:
                    print("ERROR idx in before calc: " +str(idx))     
                try: 
                    f_projLengthPointAfter = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx+1]), normalized=True) #get the length along the line of next point for comparison
                except:
                    print("ERROR idx in after calc: " +str(idx))
            elif(nDirection == 1): #direction is towards end, calculate next needed length
                f_projLengthPointAfter = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx+1]), normalized=True)
            else: #direction is towards start, calculate next needed length
                f_projLengthPointBefore = geoTrajectory.project(geo.Point(geoTrajectory.coords[idx]), normalized=True)
                
            if (f_projLengthPointBefore <= f_projLength and f_projLengthPointAfter >= f_projLength): #check whether the projected point is bounded by the two current points
                idx += 1 #point has been identified, return after point idx (as this will be the desired heading)
                break;
            else:
                if (nDirection == 0): #first loop
                    #see whether baselink projection point is closer to after or before point and set direction to the lower one
                    if(f_projLengthPointAfter-f_projLength < f_projLengthPointBefore-f_projLength):
                        nDirection = 1
                    else:
                        nDirection = -1
                    b_DirectionFound = True #set flag
            #move along line in dependent projection
            if(nDirection == 1):
                idx += 1
                f_projLengthPointBefore = f_projLengthPointAfter  #we can reuse the previously calculated value of the after-point, as it is now the before-point
                if(idx == len(geoTrajectory.coords)-1):
                    break; #last point has been reached
            elif(nDirection == -1):
                idx -= 1
                f_projLengthPointAfter = f_projLengthPointBefore# reuse before-point s after-point

    #so far only the distance has been calculated. the side of the trajectory the baselink is located on has to be accounted for as well
    #in 2D this can be determined by the sign of the cross product of two vectors
    #helpfull link:
    #https://math.stackexchange.com/questions/274712/calculate-on-which-side-of-a-straight-line-is-a-given-point-located
    #AB is the line segment of the polyline where the baselink point has been projected to (where B is idx, and A is idx-1)
    #AP is the line from the first point to the baselink point 
    AB = np.array((trajectory_xvals[idx]-trajectory_xvals[idx-1], trajectory_yvals[idx]-trajectory_yvals[idx-1])) 
    AP = np.array((baselink_x-trajectory_xvals[idx-1], baselink_y-trajectory_yvals[idx-1]))
    f_dist = f_dist*np.sign(np.cross(AB, AP))*-1
    
    #check whether the end of the trajectory could not be reached
    #this can be the case if there is walls in the track that the laser scanners do not pick up
    if (f_projLength == 1.0 and f_distToEnd > 2 ): 
        b_OutOfRange = True
    else:
        b_OutOfRange = False
    
    return [f_dist, idx, f_distToEnd, b_OutOfRange]

#returns list of x and y values of a paths pose.positions
def PathToValues(trajectory_poses):
    xvals = []
    yvals = []
    for pose in trajectory_poses:
        xvals.append(pose.pose.position.x)
        yvals.append(pose.pose.position.y)
    return [xvals, yvals]
    # return [f_dist, f_heading]

#    