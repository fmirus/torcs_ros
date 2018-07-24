#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Jul 24 09:47:42 2018

@author: bzorn
"""


#values taken form c++ controller
def SetGearFromLUT(gear, rpm):
    gearUp = [5000,6000,6000,6500,7000,0]
    gearDown = [0,2500,3000,3000,3500,3500]
    if(gear < 1):
        return 1
    if(gear <6 and rpm > gearUp[(gear-1)]):
        return gear +1
    else:
        if (gear > 1 and rpm < gearDown[(gear-1)]):
            return gear -1
        else:
            return gear