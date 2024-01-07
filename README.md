# iiwa14_Dynamics
Implements forward dynamics on KUKA LBR iiwa14 manipulator

------
## Introduction
### Robot_Dynamics
#### [iiwa14DynBase.py](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/Robot_Dynamics/src/Robot_Dynamics/iiwa14DynBase.py) contains common methods needed to compute the equation of motion
#### [iiwa14Dyn.py](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/Robot_Dynamics/src/Robot_Dynamics/iiwa14Dyn.py) implements manually coded functions used to compute the elements of the equation of motion.
#### [iiwa14DynKDL.py](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/Robot_Dynamics/src/Robot_Dynamics/iiwa14DynKDL.py) contains functions used to compute the elements of the equation of motion with the KDL library.
### iiwa_Dynamics
#### [iiwa14_Dynamics.py](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/iiwa14_Dynamics/src/iiwa14_Dynamics.py) computs and plots the joint accelerations throughout the trajectory defined in the bagfile [bag/traj.bag](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/iiwa14_Dynamics/bag/traj.bag) using dynamic components, which are computed with functions defined in [iiwa14DynKDL.py](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/Robot_Dynamics/src/Robot_Dynamics/iiwa14DynKDL.py).

------
## Execution Instruction
- git clone all the packages in ROS workspace
- change to the catkin workspace directory and run:
```commandline
catkin_make
```
- Launch the file [iiwa14_Dynamics.launch](https://github.com/alstondu/iiwa14_Dynamics/blob/main/iiwa_Dynamics/iiwa14_Dynamics/launch/iiwa14_Dynamics.launch) to run the simulation.
```commandline
roslaunch iiwa14_Dynamics iiwa14_Dynamics.launch
```

------
## Result
The 'Joint Accelerations over Time' figure below is plotted 20s after the simulation starts:
  <div align="center">
    <img width="60%" src="https://github.com/alstondu/iiwa14_Dynamics/blob/main/Figure/a_vs_t.png"></a>
  </div>
