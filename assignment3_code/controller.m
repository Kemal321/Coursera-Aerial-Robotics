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
kp_all = [30 20 30]';
kd_all = [6 5 4]';
kp_phi = 300;
kd_phi = 14;
kp_theta = 310;
kd_theta = 12;
kp_psi = 320;
kd_psi = 14;

% Accelerations
r_des_dotdot = des_state.acc + kd_all .* (des_state.vel - state.vel) + kp_all .* (des_state.pos - state.pos);

% Desired angles
phi_des = (1 / params.gravity) * (r_des_dotdot(1) * sin(des_state.yaw) - r_des_dotdot(2) * cos(des_state.yaw));
theta_des = (1 / params.gravity) * (r_des_dotdot(1) * cos(des_state.yaw) + r_des_dotdot(2) * sin(des_state.yaw));
psi_des = des_state.yaw;

% Thrust
F = params.mass * (params.gravity + r_des_dotdot(3));

% Moment
M = params.I*[kp_phi * (phi_des - state.rot(1)) + kd_phi * (0 - state.omega(1));
     kp_theta * (theta_des - state.rot(2)) + kd_theta * (0 - state.omega(2));
     kp_psi * (psi_des - state.rot(3)) + kd_psi * (des_state.yawdot - state.omega(3))];
% =================== Your code ends here ===================

end
