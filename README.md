# OAMPC

Orientation-Aware Model Predictive Control (OA-MPC) is a MATLAB simulation and control framework for dynamic bipedal robots. OA-MPC optimizes the next footstep location together with the torso rotational dynamics by linearing the augmented single rigid body model. This formulation shows improved robustness against external disturbances comapred with control methods that decouples the torso orientation dynamics and footstep location selection.

![OAMPC](https://github.com/user-attachments/assets/d78cce71-9473-4f91-9ca4-3f304da9f2d2)

Video: https://youtu.be/a8FbjdDHFRY

## Requirement
Basic: MATLAB and MATLAB optimization toolbox
OCP formulation: CasADi

## Installation
Download CasADi for your corresponding Operation System
add CasADi path to the top of the MAIN function

## Usage
navigate to the root directory and run the MAIN.m function
```MATLAB
MAIN
```

## How to Cite
    @inproceedings{ding2022orientation,
    title={Orientation-aware model predictive control with footstep adaptation for dynamic humanoid walking},
    author={Ding, Yanran and Khazoom, Charles and Chignoli, Matthew and Kim, Sangbae},
    booktitle={2022 IEEE-RAS 21st International Conference on Humanoid Robots (Humanoids)},
    pages={299--305},
    year={2022},
    organization={IEEE}}

## Acknowledgments
* Thanks to co-authors and mentors
