import numpy as np
from scipy.integrate import odeint
from scipy.linalg import block_diag
from numpy import block
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


# References ------------------------------------------
#[1] Powers, Mellinger, Kumar 2014 

# -----------------------------------------------------
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
I  = np.array([[Ixx, 0.0, 0.0],[0.0, Iyy, 0.0],[0.0, 0.0, Izz]])
Iinv = np.array([[1/Ixx, 0.0, 0.0],[0.0, 1/Iyy, 0.0],[0.0, 0.0, 1/Izz]])




# simulation parameters
tf = 5
tstep = 0.01
t = np.arange(start=0, stop=tf,step=tstep)

# linearized system matrices
A1 = np.hstack((np.zeros((6,6)),np.eye(6,6)))
A2 = np.zeros((6,12))
A2[0,4] = g
A2[1,3] = -g

A = np.vstack((A1,A2))

B1 = np.zeros((8,4))
B2 = np.array([1/m,0,0,0])
B3 = np.hstack((np.zeros((3,1)),block_diag([1/Ixx],[1/Iyy],[1/Izz])))

B = np.vstack((B1,B2,B3))

Ku = 0
Tu = 0


# position and velocity gains 
Kp = {
    'x': 2e1,
    'y': 2e1,
    'z': 1e2,
    'phi': 1e0,
    'theta': 1e0,
    'psi': 0.0
}   
Kd = {
    'x_dot': 6e0,
    'y_dot': 6e0,
    'z_dot': 1e1,
    'phi_dot': 1e-2,
    'theta_dot': 1e-2,
    'psi_dot': 0.0
} 


# equillibrium input
u0 = np.array([[m*g],[0],[0],[0]])

# initial pose
q0 = [15, 15, 15, np.pi/180*0.0, np.pi/180*0.0, np.pi/180*0.0]
q0_dot = [0]*6
x0 = np.array(q0 + q0_dot)

# desired pose
qd = [15, 15, 0, 0.0, 0.0, 0.0]
qd_dot = [0]*6
qd_ddot = [0]*6
xd = np.array(qd + qd_dot + qd_ddot)

# dynamics & control of planar quadrotor
def xdot_3d(x,t,xd):
    # CONTROLLER 
    e = xd[0:12] - x
    # position controller
    x_ddot_c = xd[12] + Kd['x_dot']*e[6] + Kp['x']*e[0]
    y_ddot_c = xd[13] + Kd['y_dot']*e[7] + Kp['y']*e[1]
    z_ddot_c = xd[14] + Kd['z_dot']*e[8] + Kp['z']*e[2]
    psi_c = xd[5]
    phi_c = 1/g*(x_ddot_c*np.sin(psi_c) - y_ddot_c*np.cos(psi_c))
    theta_c = 1/g*(x_ddot_c*np.cos(psi_c) - y_ddot_c*np.sin(psi_c))
    #u1 = z_ddot_c + m*g        #NL
    u1 = z_ddot_c 
    
    # attitude controller
    p = x[9]
    q = x[10]
    r = x[11]
    p_c = 0.0   # as per [1]
    q_c = 0.0   
    r_c = 0.0   
    u2 = np.array([Kp['phi']*(phi_c - x[3]) +     Kd['phi_dot']*(p_c - p),
                   Kp['theta']*(theta_c - x[4]) + Kd['theta_dot']*(q_c - q),
                   Kp['psi']*(psi_c - x[5]) +     Kd['psi_dot']*(r_c - r)])

    uu = np.vstack((u1,u2.reshape(3,1)))

    # NL Dynamics
    #r_dot = x[6:9]
    #omega_dot_w = body2world_vel(x[3:6],x[9:12])
    #r_ddot = np.squeeze(np.array([[0.0],[0.0],[-m*g]]) + Rot(x[3:6])@np.array([[0.0],[0.0],[u1]]))
    #omega_ddot_b = Iinv@(u2 - np.cross(x[9:12],I@x[9:12]))
    #breakpoint()

    # Linear Dynamics
    return np.squeeze(np.matmul(A,x.reshape(12,1)) + np.matmul(B,uu))

def world2body_vel(euler,omega_dot_w):
    # euler = [phi, theta, psi] the roll, pitch, and yaw angles in world frame
    # euler_dot = [phi_dot theta_dot psi_dot] world frame angular velocities
    T = np.array([[np.cos(euler[1]) , 0.0 , -np.cos(euler[0])*np.sin(euler[1])],
                  [0.0, 1, np.sin(euler[0])],
                  [np.sin(euler[1]), 0.0, np.cos(euler[0]*np.cos(euler[1]))]])
    return T@omega_dot_w

def body2world_vel(euler,omega_b):
    T = np.array([[np.cos(euler[1]) , 0.0 , -np.cos(euler[0])*np.sin(euler[1])],
        [0.0, 1, np.sin(euler[0])],
        [np.sin(euler[1]), 0.0, np.cos(euler[0]*np.cos(euler[1]))]])
    TT = np.linalg.inv(T)
    return TT@omega_b

def Rot(euler):
    # rotation matrix from body to world frame 
    R = np.array([[np.cos(euler[1])*np.cos(euler[2]) - np.sin(euler[0])*np.sin(euler[1])*np.sin(euler[2]), -np.cos(euler[0])*np.sin(euler[2]), np.cos(euler[2])*np.sin(euler[1]) + np.cos(euler[2])*np.sin(euler[0])*np.sin(euler[2])],
                 [np.cos(euler[1])*np.sin(euler[2]) + np.cos(euler[1])*np.sin(euler[1])*np.sin(euler[0]), np.cos(euler[0])*np.cos(euler[2]), np.sin(euler[1])*np.sin(euler[2]) - np.cos(euler[2])*np.cos(euler[1])*np.sin(euler[0])],
                 [-np.cos(euler[0])*np.sin(euler[1]), np.sin(euler[0]), np.cos(euler[0])*np.cos(euler[1])]])
    return R

# solve ODE
x = odeint(xdot_3d,x0,t,args=(xd,))

# extract for plotting
X = np.array(x)[:,0]
Y = np.array(x)[:,1]
Z = np.array(x)[:,2]
Phi = np.array(x)[:,3]
Theta = np.array(x)[:,4]
Psi = np.array(x)[:,5]

# plot results
#fig = plt.figure()
#ax = plt.axes(projection='3d')
#ax.plot3D(X,Y,Z)
#plt.xlabel(r'$x(t)$')
#plt.ylabel(r'$y(t)$')
#plt.zlabel(r'$z(t)$')
plt.plot(t,Z,'g')
plt.plot(t,X,'b')
plt.plot(t,Y,'r')
#plt.plot(t,Phi,'w')
#plt.plot(t,Theta,'c')
#plt.plot(t,Psi,'y')
plt.grid(linestyle='--', linewidth='0.5', color='white')

# Data for a three-dimensional line
fig2 = plt.figure(2)
ax = plt.axes(projection='3d')
ax.plot3D(X, Y, Z, 'gray')
plt.show()

