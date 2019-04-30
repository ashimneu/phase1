import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt

# function that returns 
def xdot_2d(y,t,yd):
    g =  9.80665            #[meters/sec^2]
    m = 0.030               #[kilograms]
    l = 0.046               #[meters]
    Ixx = 1.43e-5           #[kilogram*meters^2]
    Iyy = 1.43e-5           #[kilogram*meters^2]
    Izz = 2.89e-5           #[kilogram*meters^2]
    kF = 6.11e-8            #[Newton/(rpm)^2]
    kM = 1.59e-9            #[(Newton*meter)/(rpm)^2]
    gamma = kM/kF           #[meter]
    Kd = [1,1,1]
    Kp = [1,1,4]

    # equillibrium input
    u0 = np.array([[m*g],[0]])

    # position controller
    e = yd[0:5] - y
    u1 = m*(g + yd[7] + Kd[1]*e[5] +Kp[1]*e[1])
    theta_d = -1/g*(yd[6] + Kd[0]*e[3] + Kp[0]*e[0])

    # attitude controller
    u2 = Ixx*(yd[8] + Kd[2]*e[5] + Kp[2]*e[2])

    # dynamics as input-affine nonlinear system
    f = np.array([[y[3]], [y[4]], [y[5]],[0],[-g],[0]])
    g = np.array([[0, 0], [0, 0],[0 ,0],[-1/m*np.sin(y[2]), 0], [1/m*np.cos(y[2]), 0], [0,1/Ixx]])
    
    return np.squeeze(f + g@u).tolist()

# initial condition
y0 = 5
z0 = 5
phi0 = 0
vy0 = 0
vz0 = 0
phi_dot_0 = 0
x0 = [y0 ,z0, phi0, vy0, vz0, phi_dot_0]

# time points
t = np.linspace(0,5)
#yd = np.ones([6, len(t)]) 
yd = [5,20,0,0,0,0]

# solve ODE
x = odeint(xdot_2d,x0,t,args=(yd,))

# plot results
plt.plot(t,x)
plt.xlabel(r'$t$')
plt.ylabel(r'$x(t)$')
plt.show()