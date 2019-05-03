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
- [] Extend to 3d as per [Coursera Lecture](https://www.coursera.org/learn/robotics-flight/lecture/zpCD1/3-d-quadrotor-control)
### ROS
- [] Decide on topic names and message types
    - [] Confirm ROS interfaces for CrazyFly (topics,messages, services, etc.)
- [] Decide on how user inputs desired pose/trajectory
- [] Decide on how to loop integration continuously
    
  
