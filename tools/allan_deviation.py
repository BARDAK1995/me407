#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue May 12 03:36:10 2020

@author: mert
"""
actuallist=list()
import numpy as np
import allantools
import pylab as plt


file_object  = open("maviekran_datalog", "r")

mylist = list()

while True:
    a = file_object.readline().split()
#    print(a)
    
    if a is None or len(a) == 0 or a is EOFError:
        break
    else:
       mylist.append(a)

#yaw=np.zeros(len(mylist))
#roll=np.zeros(len(mylist))
#pitch=np.zeros(len(mylist))
yaw=[]
roll=[]
pitch=[]

for a in mylist:
    try:
        xxx=a[0].split('!')
        actuallist.append(xxx)
        yaw.append(float(xxx[0]))
        roll.append(float(xxx[1]))
        pitch.append(float(xxx[2]))
    except:
        print(a)
npYaw=np.array(yaw)
npPitch=np.array(pitch)
npRoll=np.array(roll)

(tausY, adevsY, errors, ns) = allantools.oadev(npYaw)

(tausR, adevsR, errors, ns) = allantools.oadev(npRoll)

(tausP, adevsP, errors, ns) = allantools.oadev(npPitch)


plt.loglog(tausY, adevsY,label='YAW')
plt.loglog(tausR, adevsR,label='ROLL')
plt.loglog(tausP, adevsP,label='PITCH')
plt.legend()
plt.show()

plt.plot(roll,linewidth=0.012)
plt.show()

plt.plot(pitch,linewidth=0.012)
plt.show()
