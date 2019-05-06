%% Main for "Dynamic" Obstacle Avoidance

%  SIMPLIFIED SYSTEM

close all;
clear all;

% Choice of generation/tracking method:
%   1. Differential Flatness
%   2. Feedback Linearization
%   3. Backstepping
%   4. Velocity Obstacle 
S.gt_type = 1; % DF, BS is kind of broken lol
S.gt_re = 0; % boolean for resampling of trajectory generation per timestep
% gt_re KINDA works w/ feedback linearization, NOT w/ df or bs

% Playable Parameters:
S.car_width = 1; % width of car in meters
S.car_len = 1; % length of car in meters
obs_vel = 5; % obstacle velocity
S.num_obs = 20; % number of obstacles
S.lane_num = 3; % number of lanes
S.lane_dist = 5; % distance in (y) for lanes in meters
S.end_dist = 100; % distance in (x) for exit in meters
S.exit_ang = 0.785; % exit angle in rads
sim_time = 10; % simulation time for ode + controls


% Car Parameters:
S.l = 1; 

% Boundary Conditions:
x0 = [S.car_len/2; S.lane_dist/2; 0]; % starting position
u_initial = [10; 0];
S.lastu = u_initial;
S.xf = [S.end_dist; S.lane_num*S.lane_dist; 0]; % desired end position
T = sim_time;

% Setup of initial conditions
if (S.gt_type == 1)
    x_state = x0;
elseif (S.gt_type == 2)
    S.k = [1;1;1];
    x_state = [x0;0;0];
elseif (S.gt_type == 3)
    S.k = [10;1];
    x_state=x0;
elseif (S.gt_type == 4)
end
    
%% First Plot

% Generating Obstacles:
for i = 1:S.num_obs
    y_rand = randi([1 S.lane_num]);
    x_rand = randi([-floor(S.end_dist/S.car_len) floor(S.end_dist/S.car_len)]);
    
    S.obs_vect(i).A = generate_obstacle([(S.car_len*x_rand) - (S.car_len/2);(S.lane_dist*y_rand) - (S.lane_dist/2)], S.car_len, S.car_width, obs_vel);
end

% Generating Environment:
f = figure();
%plot car
plot_car(x_state, S);
hold on
axis([0 S.end_dist 0 S.lane_num*S.lane_dist]);
axis equal;
% Adding lanes and goal
plot_lanes_goal(S);
% Adding obstacles
plot_obstacle(S);

% Generate and Plot Trajectory
[S.X, S.A] = generate_trajectories_simple(x_state, S.xf, u_initial(1), T);
plot(S.X(1,:), S.X(2,:), '-r')
hold off


dt = 0.05;
xs = zeros(3, T/dt+1);
%% Track Trajectory w/ Steering Force
gif('test.gif')
for t=0:dt:T
    index = floor(t/dt +1);
    xs(:, index) = x_state;
    for i = 1:S.num_obs
        S.obs_vect(i).A = update_obstacle(S.obs_vect(i).A, t);
    end
    dx = car_ode_vo(t, x_state, S);
    x_state = x_state + dx*dt;
end
%[ts, xs] = ode45(@car_ode_vo, [0 T], x_state, [], S);
%plot(xs(:,1), xs(:,2), '-b');

%% PLAY GIF
web('test.gif')