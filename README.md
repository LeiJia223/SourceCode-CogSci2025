# UR5 Collision-Free Trajectory Tracking with Online Jacobian/Parameter Estimation (MATLAB)

## Overview
This repository provides a MATLAB simulation for UR5 end-effector trajectory tracking with obstacle avoidance. The controller runs in discrete time and integrates:
- End-effector tracking in Cartesian space (position-level tracking with velocity-level commands)
- Online Jacobian/parameter-related estimation using `JacoPhi.m` and parameter vector `p_s`
- Distance-based obstacle avoidance using nearest-point geometry and an obstacle Jacobian
- Projection-based saturation to enforce joint velocity/acceleration bounds
- Visualization: tracking error, joint states, estimation errors, obstacle distance, and 3D animation

The main entry point is `main.m`.

## Code Structure

### Core kinematics
- `getDHParams.m`  
  Returns UR5 DH parameters given joint configuration `q`.
- `DH2T.m`  
  Standard DH transform `T(theta,d,a,alpha)`.
- `fkine.m`  
  Forward kinematics. Outputs transforms `T_all` and joint positions `o_all`.
- `computeJacobian.m`  
  Computes the translational Jacobian (3×6).
- `computeJacobianDot.m`  
  Approximates `Jdot` by finite differences along direction `u` (internally forms a 6×6 Jacobian; `main.m` uses the top 3 rows).

### Online estimation
- `JacoPhi.m`  
  Constructs basis/regressor terms (`Phis`, `Phi`, `H`) used to update the estimated parameter vector `p_s`.  
  The estimated Jacobian proxy is built as `J_cc` (stacked from 18 entries).

### Obstacle geometry and nearest-point Jacobians
- `ur5_to_box.m`  
  Computes a closest point on the obstacle to a sampled point on a selected link segment.
- `ur5_nearest_pc_jc.m`  
  Computes the closest point `pc` on the selected link segment to obstacle point `obs`, and returns the corresponding Jacobian `Jc`.
- `obcompared.m`  
  Similar closest-point and Jacobian computation, with an option to use identified geometric parameters `p_s` internally. Outputs `Jces` for forming an obstacle Jacobian.

## How to Run
1. Put all `.m` files in the same folder (project root), or add the folder to MATLAB path.
2. Open MATLAB and set the current folder to the project directory.
3. Run:
```matlab
main
