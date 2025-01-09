function [coff, A, b] = getCoff(waypoints)
    n = size(waypoints, 1) - 1; % number of segments P1..n
    A = zeros(8*n, 8*n);
    b = zeros(8*n, 1);

    row = 1;

    % Constraint 1: Pi(0) = Wi for all i=1..n
    for i = 1:n
        A(row, 8*(i-1)+1:8*i) = polyT(8, 0, 0);
        b(row) = waypoints(i);
        row = row + 1;
    end

    % Constraint 2: Pi(1) = Wi+1 for all i=1..n
    for i = 1:n
        A(row, 8*(i-1)+1:8*i) = polyT(8, 0, 1);
        b(row) = waypoints(i+1);
        row = row + 1;
    end

    % Constraint 3: P1(k)(0)=0 for all 1<=k<=3
    for k = 1:3
        A(row, 1:8) = polyT(8, k, 0);
        b(row) = 0;
        row = row + 1;
    end

    % Constraint 4: Pn(k)(1) = 0 for all 1<=k<=3
    for k = 1:3
        A(row, 8*(n-1)+1:8*n) = polyT(8, k, 1);
        b(row) = 0;
        row = row + 1;
    end

    % Constraint 5: Pi-1(k)(1) = Pi(k)(0) for all i=2..n and for all k=1..6
    for i = 2:n
        for k = 1:6
            A(row, 8*(i-2)+1:8*(i-1)) = polyT(8, k, 1);
            A(row, 8*(i-1)+1:8*i) = -polyT(8, k, 0);
            b(row) = 0;
            row = row + 1;
        end
    end

    % Solve for coefficients
    coff = inv(A) * b;
end










persistent waypoints0 traj_time d0 coffx coffy coffz
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    d0 = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    traj_time = [0, cumsum(d0)];
    waypoints0 = waypoints;
else
    if(t > traj_time(end))
        t = traj_time(end);
    end
    t_index = find(traj_time >= t,1);

    if(t_index > 1)
        t = t - traj_time(t_index-1);
    end
    if(t == 0)
        desired_state.pos = waypoints0(:,1);
    else
        scale = t/d0(t_index-1);
        desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
    end
    desired_state.vel = zeros(3,1);
    desired_state.acc = zeros(3,1);
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end
%

%% Fill in your code here

% desired_state.pos = zeros(3,1);
% desired_state.vel = zeros(3,1);
% desired_state.acc = zeros(3,1);
% desired_state.yaw = 0;
end

function [ desired_state ] = traj_generator(t, state, waypoints)