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
I  = np.array([[Ixx, 0.0, 0.0],[0.0, Iyy, 0.0],[0.0, 0.0, Izz]])
Iinv = np.array([[1/Ixx, 0.0, 0.0],[0.0, 1/Iyy, 0.0],[0.0, 0.0, 1/Izz]])

# simulation parameters
tf = 1
tstep = 0.005
t = np.arange(start=0, stop=tf,step=tstep)

# position and velocity gains 
Kp = {
    'x': 0.0,
    'y': 0.0,
    'z': 0.0,
    'phi': 0.0,
    'theta': 0.0,
    'psi': 0.0
}   
Kd = {
    'x_dot': 0.0,
    'y_dot': 0.0,
    'z_dot': 0.0,
    'phi_dot': 0.0,
    'theta_dot': 0.0,
    'psi_dot': 0.0
} 

# equillibrium input
u0 = np.array([[m*g],[0],[0],[0]])

# initial pose
q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q0_dot = [0]*6
x0 = np.array(q0 + q0_dot)

# desired pose
qd = [0.0, 0.0, 10.0, 0.0, 0.0, np.pi/2]
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
    u1 = z_ddot_c
    # attitude controller
    body = world2body_vel(x[4:6],x[9:12])
    p = body[0]
    q = body[1]
    r = body[2]
    p_c = 0.0   # as per Powers,Mellinger,Kumar 2014
    q_c = 0.0   # as per Powers,Mellinger,Kumar 2014
    r_c = 0.0   # as per Powers,Mellinger,Kumar 2014
    u2 = np.array([Kp['phi']*(phi_c - x[3]) + Kd['phi_dot']*(p_c - p),
                   Kp['theta']*(theta_c - x[4]) + Kd['theta_dot']*(q_c - q),
                   Kp['psi']*(psi_c - x[5]) + Kd['psi_dot']*(r_c - r)])
    u = np.append(u1,u2.T) + u0.T
    # CLOSED-LOOP DYNAMICS
    euler_dot = body2world_vel(euler,body)
    body_dot = Iinv*@(u2 - np.cross(body,I@body))
    lin_ddot = np.array([[0.0],[0.0],[-m*g]]) + R(euler)@np.array([[0.0],[0.0],[u1]])
    lin_dot = np.array([[],[],[]])@lin
    Xdot = np.append(lin_dot,euler_dot,lin_ddot,body_dot,axis=0)

    return np.squeeze(Xdot) 

def world2body_vel(euler,euler_dot):
    # euler = [phi, theta, psi] the roll, pitch, and yaw angles in world frame
    # euler_dot = [phi_dot theta_dot psi_dot] world frame angular velocities
    T = np.array([[np.cos(euler[1]) , 0.0 , -np.cos(euler[0])*np.sin(euler[1])],
        [0.0, 1, np.sin(euler[0])],
        [np.sin(euler[1]), 0.0, np.cos(euler[0]*np.cos(euler[1]))]])
    return T@euler_dot

def body2world_vel(euler,body):
    T = np.array([[np.cos(euler[1]) , 0.0 , -np.cos(euler[0])*np.sin(euler[1])],
        [0.0, 1, np.sin(euler[0])],
        [np.sin(euler[1]), 0.0, np.cos(euler[0]*np.cos(euler[1]))]])
    TT = np.linalg.inv(T)

    return TT@body

    

# ADD LOGIC TO EXTRACT FORCES/MOMENTS FROM MODEL

# UPDATE ODEINT USAGE TO REFLECT ROS WRAP

# solve ODE
x = odeint(xdot_3d,x0,t,args=(xd,))

# extract for plotting
X = np.array(x)[:,0]
Y = np.array(x)[:,1]
Z = np.array(x)[:,2]

# plot results
plt.plot(X,Y,Z)
plt.xlabel(r'$x$')
plt.ylabel(r'$y(t)$')
plt.zlabel(r'$z(t)$')
plt.show()