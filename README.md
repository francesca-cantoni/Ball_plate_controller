# Ball-Plate controller
Design of different basic control schemes for the Ball-Plate problem.

<p align="center">
  <img src="Ball_Plate_animation.gif">
 </p>

## Prerequisites
- MATLAB R2018b
- Simulink

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
1. Install **Simscape** and **Simscape Multibody** tools from [Add-On Explorer](https://it.mathworks.com/products/matlab/add-on-explorer.html)
2. Run **Ball_plate_library.m**
3. Open one of the Simulink schemes and test the performance of the implemented controller 

## Autor
**Francesca Cantoni:** 	francescacantoni95@gmail.com
