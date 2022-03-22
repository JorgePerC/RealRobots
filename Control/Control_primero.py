"""
CONTROL PID
BEFORE RUNNING:
    Create a ve to run and install the libraries
"""

import numpy as np
import control 
from scipy import signal 
import math
import matplotlib.pyplot as plt


# Define robot parameters
mass = 1    # kg
length = 1  # m
lc = 0.5    # Distance to center of mass
B = 0.01    # Viscosity friction coefficient
inert = (1/3)*mass*math.pow(length, 2) # inertia
g = 9.81    # gravity

# 

K = math.pi/4 # Ganancia 

# PID control params
kp = 8
kd = 0.5
ki = 0.3

# Simulation time

start = 0
stop = 5
step = 0.01
tiempo = np.arange(start, stop, step)

# Define transfer function

alfa1 = (B*math.pow(lc,2))/(mass*math.pow(lc,2) + inert)
alfa0 = (mass*g*lc)/(mass*math.pow(lc,2) + inert)

gama = 1/(mass*math.pow(lc,2) + inert)

b2 = gama*kp*ki*kd
b1 = gama*kp*ki
b0 = gama*kp

a3 = ki
a2 = ki*alfa1 + gama*kp*ki*kd
a1 = ki*alfa0 + gama*kp*ki
a0 = gama*kp

numerador = np.array([K*b2, K*b1, K*b0])
denominador = np.array([a3, a2, a1, a0])
H = control.tf(numerador, denominador)
print('T(s) =', H)

# Print graph
t, y = control.step_response(H, tiempo)
plt.plot(t, y)
plt.title("Control PID del robot")
plt.xlabel("t")
plt.ylabel("Angulo robot")
plt.grid()
plt.show()
