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

# simulation parameters
tf = 15
tstep = 0.01

# position and velocity gains 
Kp = [0.1,50,1e2]   #[0,50,0]
Kd = [0.2,10,1e1]   #[0,10,0]     

# equillibrium input
u0 = np.array([[m*g],[0]])

# function that returns 
def xdot_2d(y,t,yd):
    # CONTROLLER 
    # position controller
    e = yd[0:6] - y
    u1 = m*(yd[7] + Kd[1]*e[4] +Kp[1]*e[1])
    theta_d = -1/g*(yd[6] + Kd[0]*e[3] + Kp[0]*e[0])
    # attitude controller
    u2 = Ixx*(yd[8] + Kd[2]*e[5] + Kp[2]*(theta_d - y[2]))# add theta_d 
    u = np.array([[u1],[u2]]) + u0

    # dynamics
    F = np.array([[y[3]], [y[4]], [y[5]],[0],[-g],[0]])
    G = np.array([[0, 0], [0, 0],[0 ,0],[-1/m*np.sin(y[2]), 0], [1/m*np.cos(y[2]), 0], [0,1/Ixx]])

    return np.squeeze(F + G@u).tolist() # update to python 2.7

# initial condition
y0 = 0
z0 = 0
phi0 = 1.0
vy0 = 0
vz0 = 0
phi_dot_0 = 0
x0 = [y0 ,z0, phi0, vy0, vz0, phi_dot_0]

# time points
t = np.arange(start=0, stop=tf,step=tstep)

# desired pose
yd = [0,0,0,0,0,0,0,0,0]

# solve ODE
x = odeint(xdot_2d,x0,t,args=(yd,))

# extract for plotting
Y = np.array(x)[:,0]
Z = np.array(x)[:,1]
Theta = np.array(x)[:,2]

# plot results
plt.plot(t,Y,'r')
plt.plot(t,Z,'b')
plt.plot(t,Theta,'g')
plt.xlabel(r'$t$')
plt.ylabel(r'$x(t)$')
plt.show()