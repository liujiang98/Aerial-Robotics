# Aerial-Robotics
code for Aerial Robotics

[Aerial Robotics](https://www.coursera.org/learn/robotics-flight?specialization=robotics)系列课程主要是针对四旋翼无人机的课程，[b站](https://www.bilibili.com/video/BV1K7411n7L3)上也有搬运。整个课程可以对四旋翼的运动学、动力学和PID控制有一个基本的了解。

比较有意思的是课后的几个大作业，认真写一写可以加深对PID的理解。这里直接把代码和控制效果放上来

# 2-D Quadrotor Control

![2D.gif (778×591) ](https://raw.githubusercontent.com/heoll0/Aerial-Robotics/main/2-D Quadrotor Control/2D.gif)

```matlab
function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

%line
% Kpz = 80;
% Kvz = 5;
% Kpy = 10;
% Kvy = 1;
% Kpf = 400;
% Kvf = 20;

%sine
Kpz = 80;
Kvz = 3;
Kpy = 20;
Kvy = 5;
Kpf = 400;
Kvf = 20;

u1 = params.mass * (params.gravity + des_state.acc(2) + Kvz*(des_state.vel(2) - state.vel(2)) + Kpz*(des_state.pos(2) - state.pos(2)));
fc = -1/params.gravity * (des_state.acc(1) + Kvy*(des_state.vel(1) - state.vel(1))+ Kpy*(des_state.pos(1) - state.pos(1)));
u2 = params.Ixx * (Kvf*(0-state.omega) + Kpf*(fc-state.rot));


end
```



# 3-D Quadrotor Control

![3D](https://raw.githubusercontent.com/heoll0/Aerial-Robotics/main/3-D Quadrotor Control/3D.gif)



```matlab
function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
kpf = [60; 60; 60];
kdf = [6; 6; 6];
des_rddot = des_state.acc + kdf.*(des_state.vel - state.vel) + kpf.*(des_state.pos - state.pos);
F = params.mass*(params.gravity + des_rddot(3));

% Moment
kpm = [20; 20; 20];
kdm = [1; 1; 1];
phi_des = (des_rddot(1)*sin(des_state.yaw) - des_rddot(2)*cos(des_state.yaw))/params.gravity;
theta_des = (des_rddot(1)*cos(des_state.yaw) + des_rddot(2)*sin(des_state.yaw))/params.gravity;
des_rot = [phi_des; theta_des; des_state.yaw];
des_omega = [0; 0; des_state.yawdot];
M = kpm.*(des_rot - state.rot) + kdm.*(des_omega - state.omega);


end
```

