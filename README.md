# Project Phase1
First phase of EE 245 Advanced Robotics project. The goal is to derive and implement stabilizing controllers for the 2d and 3d quadrotor models to achieve vertical position hover control.

## Tasks
### 2D
- [x] Derive 2d quadrotor dynamics and stabilizing controller
  - [x] Write 2d EOM
- [x] Propagate dynamics in Python
    - assume test inputs $u_1, u_2$ 
- [x] Find stabilizing linear feedback controller
    - [x]apply on $u_1,u_2$
    - []solve for $F_1,F_2,F_3,F_4$
- [x] Plot $y,z,\phi$
### 3D
- [] Derive the commanded euler angle accelerations $ \phi_c,\theta_c,\psi_c$ and use to solve for $\textbf{u}_2$
- [] Extend to 3d as per [Coursera Lecture](https://www.coursera.org/learn/robotics-flight/lecture/zpCD1/3-d-quadrotor-control)
### ROS
- [] Decide on topic names and message types
    - [] Confirm ROS interfaces for CrazyFly (topics,messages, services, etc.)
- [] Decide on how user inputs desired pose/trajectory
- [] Decide on how to loop integration continuously
    
  
# Notes
# 3D Quadrotor
- The following were obtained from the Coursera Robotics Flight Course section on [3D control](https://www.coursera.org/learn/robotics-flight/lecture/zpCD1/3-d-quadrotor-control)
## Dynamics 
- The linear dynamics of the quadrotor are define in the non-inertial frame i.e. the world frame by Newton's 2nd Law:

$m \mathbf{\ddot{r}} =\left[\begin{array}{c}
        0\\
        0\\
        -mg
    \end{array}\right] + R \left[
        \begin{array}{c}
            0\\
            0\\
            u_1
        \end{array}
    \right]$
- the rotational dynamics in the body frame are given by 
$I \dot{\left[\begin{array}{c}
        p\\
        q\\
        r
    \end{array}\right]} -\mathbf{u_2} - \left[
        \begin{array}{c}
            p\\
            q\\
            r
        \end{array}
    \right] \times 
    I\left[
        \begin{array}{c}
            p\\
            q\\
            r
        \end{array}
    \right]
$
## Controller Design
- We desire that the quadrotor tracks a desired trajectory with zero error:
$(\ddot{r}_d(t) -\ddot{r}_c) + K_de_v + K_pe_p = 0 \quad (1)$
- where our control authority over the propellers allows us to command the quadrotor's acceleration $\ddot{r}(t)$.
- we obtain feedback on the position $r(t)$ and velocity $\dot{r}(t)$ and from (1) can design a cascading  position $\rightarrow$ attitude controller:
- $\phi_c = \frac{1}{g}(\ddot{x}_c \sin{\psi}_{des} - \ddot{y}_c \cos{\psi}_{des})\\
\theta_c =\frac{1}{g}(\ddot{x}_c \cos{\psi}_{des} + \ddot{y}_c \sin{\psi}_{des})\\
\psi_c = \psi_{des}\\
u_1 = m(g+\ddot{z}_c)
$
where $\ddot{x}_c, \ddot{y}_c$, and $\ddot{z}_c$ are obtained from $(1)$, which completes the position controller. The attitude controller follows as
$
u_2 = \left[\begin{array}{c}
        k_{p,\phi}(\phi_c - \phi) + k_{d,\phi}(p_c - p)\\
        k_{p,\theta}(\theta_c - \theta) + k_{d,\theta}(q_c - q)\\
        k_{p,\psi}(\psi_c - \psi) + k_{d,\psi}(r_c - r) 
    \end{array}\right]
$
where the angular rates in the body frame are related to the world frame rates by
$\left[
    \begin{array}{c}
        p\\
        q\\
        r
    \end{array}
\right]
=
\left[
    \begin{array}{ccc}
        c_{\theta} & 0 & -c_{\phi}s_{\theta}\\
        0 & 1 & s_{\phi} \\
        s_{\theta} & 0 & c_{\phi}c_{\theta}
    \end{array}
\right]
\left[
    \begin{array}{c}
        \dot{\phi}\\
        \dot{\theta}\\
        \dot{\psi}
    \end{array}
\right] \quad (2)
$
- **Q:** How are the commanded body-frame angular rates $p_c,q_c,r_c$ obtained?
    - Can we re-use the transform $(2)$, and if so, how are the commanded euler rates obtained?
    - From Vijay's lecture, it is pointed out that in the vicinity of the hover condition (operating point ~ linear regime), the euler rates and body angular rates are almost equivalent: $\dot{\phi} \approx p,\dot{\theta} \approx q,\dot{\psi} \approx r$
    - furthermore, the commanded body angular rates can be obtained by from $(2)$ with inputs being the commanded euler angles and their *time-derivatives* 
