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

% =================== Your code ends here ===================

end
