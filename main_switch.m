%% Main for "Dynamic" Obstacle Avoidance
close all;
clear all;

% Choice of generation/tracking method:
%   1. Differential Flatness
%   2. Feedback Linearization
%   3. Backstepping
%   4. Velocity Obstacle 
S.gt_type = 3; % DF, BS is kind of broken lol
S.gt_re = 0; % boolean for resampling of trajectory generation per timestep
% gt_re KINDA works w/ feedback linearization, NOT w/ df or bs

% Playable Parameters:
S.car_width = 1; % width of car in meters
S.car_len = 1; % length of car in meters
obs_vel = 10; % obstacle velocity
S.num_obs = 50; % number of obstacles
S.lane_num = 3; % number of lanes
S.lane_dist = 5; % distance in (y) for lanes in meters
S.end_dist = 100; % distance in (x) for exit in meters
S.exit_ang = 0.785; % exit angle in rads
S.t_offset = 0;
sim_time = 11; % simulation time for ode + controls


% Car Parameters:
S.l = 1; 

% Boundary Conditions:
x0 = [S.car_len/2; S.lane_dist/2; 0]; % starting position
u_initial = [10; 0];
S.xlane = [S.end_dist; 0.5*S.lane_dist; 0]; % First lane
S.xm(1).m = [S.end_dist; 1.5*S.lane_dist; 0]; % Second lane
S.xm(2).m = [S.end_dist; 2.5*S.lane_dist; 0]; % Third lane
S.xf = [S.end_dist; (S.lane_num+0.5)*S.lane_dist; 0]; % desired end position
S.xm(3).m = [S.xf(1)+20; S.xf(2); S.xf(3)];
S.new_goal = [0;0;0]; % for switching purposes

S.T = sim_time;
% Setup of initial conditions
if (S.gt_type == 1)
    x_state = x0;
    % u1 is velocity here
    S.u_range = [1;.3];
elseif (S.gt_type == 2)
    S.k = [1;1;1];
    x_state = [x0;10];
    % u1 is accleration here
    S.u_range = [3;.3];
    u_initial = [0;0];
elseif (S.gt_type == 3)
    S.k = [1;1];
    x_state=[x0; 10];
    % u1 is accleration here
    S.u_range = [3;.3];
    u_initial = [0;0];
end
S.lastu = u_initial;
%% First Plot

% Generating Obstacles:
for i = 1:S.num_obs
    y_rand = randi([1 S.lane_num]);
    x_rand = 0;
    % dont spawn obstacles too close to car
    while (abs(x_rand) < 5) 
        x_rand = randi([-floor(S.end_dist/S.car_len) floor(S.end_dist/S.car_len)]);
    end
%     v_rand = [obs_vel + rand*10 - 5; 0];
%     theta_rand = rand*0.3 - .15;
    v_rand = obs_vel;
    theta_rand = 0;
    v_rand = rot(theta_rand)*v_rand;
    S.obs_vect(i).A = generate_obstacle([(S.car_len*x_rand) - (S.car_len/2);(S.lane_dist*y_rand) - (S.lane_dist/2)], S.car_len, S.car_width, v_rand);
end

% Generating Environment:
f = figure();
hold on
axis([-S.end_dist S.end_dist 0 S.lane_num*S.lane_dist]);
axis equal;

%plot car
plot_car(x_state, S);
% Adding lanes and goal
plot_lanes_goal(S);
% Adding obstacles
plot_obstacle(S);
% Generate and Plot Trajectory
[S.X, S.A] = generate_trajectories_simple(x_state, S.xlane, 10, S.T);
plot(S.X(1,:), S.X(2,:), '-r')
hold off

S.switching = 0;
S.switch_count = 0;
dt = 0.05;
%% Track Trajectory w/ Steering Force
gif('test.gif', 'DelayTime', dt);
us = S.lastu;
for t=0:dt:S.T
    index = floor(t/dt +1);
    xs(:, index) = x_state;
    [dx,S,us] = car_ode_vo_switch(t, dt, x_state, S, us);
    x_state = x_state + dx*dt;
end
%[ts, xs] = ode45(@car_ode_vo, [0 T], x_state, [], S);
%plot(xs(:,1), xs(:,2), '-b');
us = us(:,1:end-1);

% %% Plot Controls
T = 0:dt:S.T;
figure;
plot(T, us(1,:) , '.');
title('u1')
xlabel('time')
ylabel('u1 (acceleration in m/s)')
figure;
plot(T, us(2,:), '.');
title('u2')
xlabel('time')
ylabel('u2 (steering angle)')

%% PLAY GIF
web('test.gif')