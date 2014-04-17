#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math
dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    #Set time which will be used to standardize the frequency of the system
    otime2=tim.sim[0]
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
    #Set error to 0 
    err=0
    if(i==0):
	starttime=tim.sim[0]
	f = open("data2.txt", "w")
	errarray=[0];
	enginevalL=0
	enginevalR=0
    i=i+1

    check=0
    #Above:Set a variable which will be used to determine whether the object we are looking for is in the frame
    #Below:Find the Object in the frame that is green
    blue = cv2.inRange(imgR, np.array([0,0,0], dtype = np.uint8), np.array([0,255,0], dtype = np.uint8));
    blue2 = cv2.inRange(imgL, np.array([0,0,0], dtype = np.uint8), np.array([0,255,0], dtype = np.uint8));
    #Below: Find the contours for the object in the left and right cameras
    cntRGB, h = cv2.findContours(blue, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    cntRGB2, h2 = cv2.findContours(blue2, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    #Below: set variables to zero
    check=0
    distance=0
    radius1=0
    radius2=0
    x=0
    x2=0
    #Go through the frame and put an enclosing circle around the contours for both video feeds
    for cnt in cntRGB:
	(x, y), radius = cv2.minEnclosingCircle(cnt)
	center = (int(x), int(y))
	radius1 = int(radius)
	check=1
	print 'x, y ', x, y
    #Go through the frame and put an enclosing circle around the object in view
    for cnt2 in cntRGB2:
	(x2, y2), radius2 = cv2.minEnclosingCircle(cnt2)
	center2 = (int(x2), int(y2))
	radius2 = int(radius2)
	check=1
	print 'x2, y2', x2,y2
    #If there is an object in the frame continue
    if(check==1 and radius1<100 and radius2<100):
	#Determine the distance from the ball in meters
	if(x2!=x):
    		distance=(.4*320)/(2*(math.tan(1.047/2))*abs(x2-x))
	#Subtract the distance from center in terms of the first camera
    	err = (nx/2) - x;
	err2= distance-4
	print 'Error from box(4 meters): ',err2
	#return the error 1 values
    	print 'error in pixels = ',err
	print 'error in percent = ', err/640

	#Append to the error array which will be summilarily summed and be used as a velocity controller	
	errarray.append(err2)
	print 'Error From: ',sum(errarray)
	print radius1
	print radius2
	#Apply gains to errors and send the values to the engine values
    	ref.ref[0]=err*.009+.008*sum(errarray)
	ref.ref[1]=-1*err*.009+.008*sum(errarray)
	enginevalL=err*.009+.008*sum(errarray)
	enginevalR=-1*err*.009+.008*sum(errarray)
    #If no object is in frame keep searching
    elif(check==1):
	ref.ref[0] = enginevalL
    	ref.ref[1] = enginevalR
    else:
	ref.ref[0] = -0.35
    	ref.ref[1] = 0.35
    #print the sim,clock,and engine values
    #print 'Sim Time = ', tim.sim[0]
    #print 'Clock time=', time.clock
    print 'Engine Values', ref.ref[0],ref.ref[1]
    #Set the engine values determined above
    r.put(ref);

    # Sleeps
    #Sets frequency to 20Hz even if the code executes faster
    while((tim.sim[0]-otime2)<.05):
	[status, framesize] = t.get(tim, wait=False, last=True)
    timer=tim.sim[0]-otime2
    #prints the time elapsed and the error to a txt document

#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
