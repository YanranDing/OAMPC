% basic OA-MPC for biped using single rigid body
% features: use casadi for problem formulation
% Author: Yanran Ding
% Last modified: 2025/02
% Citation:
% @inproceedings{ding2022orientation,
%   title={Orientation-aware model predictive control with footstep adaptation for dynamic humanoid walking},
%   author={Ding, Yanran and Khazoom, Charles and Chignoli, Matthew and Kim, Sangbae},
%   booktitle={2022 IEEE-RAS 21st International Conference on Humanoid Robots (Humanoids)},
%   pages={299--305},
%   year={2022},
%   organization={IEEE}
% }

addpath(genpath('casadi_3.6.7_osx_arm64'))
addpath(genpath('fcns_common'));
tic

clear all;clc; close all;clear mex;
import casadi.*

%% initialization
MPC = OAMPC_Class;

p = MPC.init_MPC();

p.playBackSpeed = 1;
p.SimDuration = 2;

%% MAIN Loop

p = MPC.MAIN_LOOP(p);

%% Animation
animateRobot_SRB(p);
