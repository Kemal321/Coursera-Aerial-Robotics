% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

function [ desired_state ] = traj_generator(t, state, waypoints)
    persistent coffx coffy coffz traj_time d0 waypoints0
    if nargin > 2
        % Initialization
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;

        % Calculate distances between waypoints
        d = waypoints(:,2:end) - waypoints(:,1:end-1);
        d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
        
        % Define absolute time vector
        traj_time = [0, cumsum(d0)];
        waypoints0 = waypoints;

        % Find polynomial coefficients using getCoff function
        coffx = getCoff(waypoints(1,:)');
        coffy = getCoff(waypoints(2,:)');
        coffz = getCoff(waypoints(3,:)');
    else
        if t >= traj_time(end)
            t = traj_time(end) - 0.0001;
        end
        t_index = find(traj_time > t, 1) - 1;
        t_index = max(t_index, 1);
        scale = (t - traj_time(t_index)) / d0(t_index);

        % Calculate position, velocity, and acceleration
        t0 = polyT(8, 0, scale)';
        t1 = polyT(8, 1, scale)' .* (1 / d0(t_index));
        t2 = polyT(8, 2, scale)' .* (1 / d0(t_index)^2);

        index = (t_index-1)*8 + 1 : t_index*8;

        desired_state.pos = [coffx(index)' * t0; coffy(index)' * t0; coffz(index)' * t0];
        desired_state.vel = [coffx(index)' * t1; coffy(index)' * t1; coffz(index)' * t1];
        desired_state.acc = [coffx(index)' * t2; coffy(index)' * t2; coffz(index)' * t2];
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
end