# Ball-Plate controller
Design of different types of basic control schemes for the Ball-Plate problem.

<p align="center">
  <img src="Ball_Plate_animation.gif">
 </p>

## Prerequisites
- MATLAB R2018b
- Simulink
- Simscape
- Simscape Multibody

## Contents
**Files**:
- **Ball_plate_library.m**: MATLAB file for generating all the necessary matrices for all Simulink schemes
- **Linsys.mat**: MAT-file that stores the workspace variables that fully describe the ball-plate problem


**Folders:**
- **All sensors**: contains three Simulink control schemes designed assuming to fully know the state vector of the system.
  - LQR-Based Controller with a full state space observer
  - LQR-Based Controller with a reduced state space observer
  - LQR-Based Controller with a disturbance observer in case of constant disturbance
 
- **Only two sensors**: contains three Simulink control schemes designed assuming to know only x and y position of the ball.
  - LQR-Based Controller with a full state space observer
  - LQR-Based Controller with a reduced state space observer
  - LQR-Based Controller with a disturbance observer in case of constant disturbance
 
## How to run it
1. Run **Ball_plate_library.m**
2. Open one of the Simulink schemes and test the performance of the implemented controller 

## Autor
**Francesca Cantoni:** 	francescacantoni95@gmail.com
