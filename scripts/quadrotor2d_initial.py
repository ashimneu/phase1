#!/usr/bin/env python3

import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# physical parameters
g =  9.80665            #[meters/sec^2]
m = 0.030               #[kilograms]
l = 0.046               #[meters]
Ixx = 1.43e-5           #[kilogram*meters^2]
Iyy = 1.43e-5           #[kilogram*meters^2]
Izz = 2.89e-5           #[kilogram*meters^2]
kF = 6.11e-8            #[Newton/(rpm)^2]
kM = 1.59e-9            #[(Newton*meter)/(rpm)^2]
gamma = kM/kF           #[meter]
M = np.linalg.inv(np.array([[1, 1, 1, 1],[0,l,0,-l],[-l,0,l,0],[gamma,-gamma,gamma,gamma]]))

# linearized system matrices
A = np.array([[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1],[0,0,-g,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]])
B = np.array([[0,0],[0,0],[0,0],[0,0],[1/m,0],[0,1/Ixx]])

# simulation parameters
tf = 8                 # [sec]
tstep = 0.005           # [sec]
t = np.arange(start=0, stop=tf,step=tstep)

# position and velocity gains 
Ku = 1500
Kp = [1,50,0.8*Ku]   #[20,50,1e3] 
Kd = [1.5,10,Kp[2]/8]      #[Kp[0]/3,10,1e2]    

# equillibrium input
u0 = np.array([[m*g],[0]])

# initial condition: take-off from rest at origin
y0 = -200.0
z0 = 200.0
phi0 = 0.0
vy0 = 0.0
vz0 = 0.0
phi_dot_0 = 0.0
x0 = np.asarray([y0 ,z0, phi0, vy0, vz0, phi_dot_0])


# non-linear system dynamics + control
def nl_xdot_2d(y,t,yd):     
    # CONTROLLER 
    e = yd[0:6] - y
    # position controller
    u1 = m*(yd[7] + Kd[1]*e[4] +Kp[1]*e[1])
    theta_d = -1/g*(yd[6] + Kd[0]*e[3] + Kp[0]*e[0])
    # attitude controller
    u2 = Ixx*(yd[8] + Kd[2]*e[5] + Kp[2]*(theta_d - y[2]))# add theta_d 
    u = np.array([[u1],[u2]]) + u0

    # CLOSED-LOOP DYNAMICS
    F = np.array([[y[3]], [y[4]], [y[5]],[0],[-g],[0]])
    G = np.array([[0, 0], [0, 0],[0 ,0],[-1/m*np.sin(y[2]), 0], [1/m*np.cos(y[2]), 0], [0,1/Ixx]])

    return np.squeeze(F + G@u)  

# linearized system dynamics + control
def l_xdot_2d(y,t,yd):      
   
    # CONTROLLER 
    e = yd[0:6] - y
    # position controller
    u1 = m*(yd[7] + Kd[1]*e[4] +Kp[1]*e[1])
    theta_d = -1/g*(yd[6] + Kd[0]*e[3] + Kp[0]*e[0])
    # attitude controller
    u2 = Ixx*(yd[8] + Kd[2]*e[5] + Kp[2]*(theta_d - y[2]))# add theta_d 
    u = np.array([[u1],[u2]])


    # CLOSED-LOOP DYNAMICS
    ystep = A@y.reshape(6,1) + B@u    
    return np.squeeze(ystep)  

def simulate():

    #sim_type = int(input("Type '0' for non-linear simulation or '1' for linear simulation:  " ))
    sim_type=1
    # desired pose
    yd = np.asarray([10.0,10.0,0,0,0,0,0,0,0])

    if sim_type == 0:
        x = odeint(nl_xdot_2d,x0,t,args=(yd,))
    else:
        x = odeint(l_xdot_2d,x0,t,args=(yd,))


    # extract for plotting
    Y = np.array(x)[:,0]
    Z = np.array(x)[:,1]
    Theta = np.array(x)[:,2]

    # plot results
    plt.plot(t,Y,'r')
    #plt.plot(t,Z,'b')
    plt.plot(t,Theta,'g')
    plt.xlabel(r'$t$')
    plt.ylabel(r'$x(t)$')
    plt.show()

if __name__ == '__main__':
    simulate()