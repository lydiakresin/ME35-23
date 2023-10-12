# -*- coding: utf-8 -*-
"""
Created on Fri Oct  6 16:07:15 2023

@author: lkres
"""

import numpy as np
import math
from matplotlib import pyplot as plt
import time
import paho.mqtt.client as mqtt 

# ------------- INVERSE KINEMATICS CALCULATIONS --------------- #
NUMPTS = 100 # number of points along circle to calculate angles for
WAIT = 0.3

# Calculates the corresponding angles to an (x,y) point along
# a circle, with robotic arms of lengths d1 and d2
def invKinematics(x, y, d1, d2):
    L = math.sqrt(x**2 + y**2)
    gamma2 = math.acos((d1**2+L**2-d2**2)/(2*d1*L))
    gamma1 = math.acos((d1**2+d2**2-L**2)/(2*d1*d2))
    theta = math.atan2(y,x) - gamma2 # theta is angle relative to ground
    alpha = math.pi - gamma1 # alpha is angle relative to first arm
    theta = -1*math.degrees(theta)
    alpha = -1*math.degrees(alpha)
    return [theta, alpha]

# Function that finds (x, y) points along a circle given the diameter
def circlePts(d, numPts):
    numPts = int(numPts/2) # because doubling number of points to get other half of circle
    x = np.linspace(d/2, 3*d/2,numPts).tolist() # getting possible x values along circle
    y = []
    for i in range(len(x)): # getting possible y values along circle
        #print("vals: ", x[i])
        yEqn = math.sqrt((d/2)**2 - (x[i]-d)**2)
        y.append(yEqn)
    # Adding bottom half of circle
    x_reversed = list(reversed(x))
    for i in range(len(x)):
        y.append(-y[i])
        x.append(x_reversed[i])
    return x,y # x and y are lists

x_vals, y_vals = circlePts(1, NUMPTS)

# Function that returns all the corresponding angles of the robotic
# arm given points around a circle
def findAngles(x_vals, y_vals):
    reversedAngles = []
    thetavals = []
    alphavals = []
    for i in range(len(x_vals)):
        theta, alpha = invKinematics(x_vals[i], y_vals[i], 1, 1)
        thetavals.append(theta)
        alphavals.append(alpha)
        reversedAngles.append((theta, alpha))
    return reversedAngles, thetavals, alphavals

reversedAngles, thetavals, alphavals = findAngles(x_vals, y_vals)
thetavals = list(reversed(thetavals))
#print(reversedAngles)



# Function that performs forward kinematics using angles found with inverse
# kinematics to ensure that angles have been found correctly
def forwardKinematics(thetavals, alphavals, d1, d2, x_0, y_0):
    x_f = []
    y_f = []
    for i in range(len(reversedAngles)):
        x_1 = x_0 + d1 * math.cos(math.radians(thetavals[i]))
        y_1 = y_0 + d1 * math.sin(math.radians(thetavals[i]))
        x_e_check = x_1 + d2 * math.cos(math.radians(thetavals[i]) + math.radians(alphavals[i]))
        y_e_check = y_1 + d2 * math.sin(math.radians(thetavals[i]) + math.radians(alphavals[i]))
        x_f.append(x_e_check)
        y_f.append(y_e_check)
    return x_f, y_f

x_f, y_f = forwardKinematics(thetavals, alphavals, 1, 1, 0, 0)
#print(x_f, y_f)

# ------------- PLOTTING GRAPHS --------------- #
plt.figure()
plt.title("Equation of circle")
plt.plot(x_vals, y_vals) #plot points to confirm drawing circle correctly
plt.axis('scaled')

plt.figure()
plt.title("Forward kinematics of inverse kinematics vals")
plt.plot(x_f, y_f, color = "green") #plot points to confirm drawing circle correctly
plt.axis('scaled')

# ------------- CONNECTING TO MQTT --------------- #

broker_address='192.168.1.16' # change to Chris' IP address

client = mqtt.Client("Lydia") 
client.connect(broker_address) 

# Publishing angles one by one, in (theta, alpha) pairs
for angles in reversedAngles:
    client.publish("test", str(angles)) # change topic to Chris' topic
    time.sleep(WAIT)
client.disconnect()