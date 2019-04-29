%% Main for "Dynamic" Obstacle Avoidance
close all;
clear all;
% Playable Parameters:
car_width = 1; % width of car in meters
car_len = 6; % length of car in meters
obs_vel = 1; % obstacle velocity
num_obs = 5; % number of obstacles
lane_num = 3; % number of lanes
lane_dist = 5; % distance in (y) for lanes in meters
end_dist = 100; % distance in (x) for exit in meters
exit_ang = 0.785; % exit angle in rads
sim_time = 20; % simulation time for ode + controls
S.k(1) = 1;
S.k(2) = 2;
S.l = car_width;


% Car Parameters:
l = car_width;

% Boundary Conditions:
x0 = [car_len/2; lane_dist/2; 0]; % starting position
xf = [end_dist; lane_num*lane_dist; exit_ang]; % desired end position
T = sim_time;


% Choice of generation/tracking method:
%   1. Differential Flatness
%   2. Feedback Linearization
%   3. Backstepping
%   4. Velocity Obstacle 
gt_type = 3;
gt_re = 0; % boolean for resampling of trajectory generation per timestep
if (gt_type == 1)
elseif (gt_type == 2)
elseif (gt_type == 3)
    x_state = [x0;0];
elseif (gt_type == 4)
end
    
%% First Plot
% Generating Obstacles:
for i = 1:num_obs
    y_rand = randi([1 3]);
    x_rand = randi([2 floor(end_dist/car_len)]);
    obs_vect(i).A = generate_obstacle([(car_len*x_rand) - (car_len/2),(lane_dist*y_rand) - (lane_dist/2)], car_len, car_width, obs_vel);
end

% Generating Environment:
f = figure();
%plot car
plot_car(x_state, car_len, car_width);

% Adding lanes and goal
plot_lanes_goal(lane_num, lane_dist, xf, end_dist)

% Adding obstacles
for i = 1:num_obs
    plot_obstacle(obs_vect(i).A);
end


% Generate and Plot Trajectory
[X, dX, A] = generate_trajectories(x0, xf, 1, T);
S.A = A;

plot(X(1,:), X(2,:), '-r')
hold off


gif('test.gif')
dt = 0.05;
times = 0:dt:T;
time_offset = 0; 
for n=1:size(times,2)
    cla 
    hold on
    t = times(n) - time_offset;
    % Plot planes and goal
    plot_lanes_goal(lane_num, lane_dist, xf, end_dist)
    
    % Update and Plot Obstacles
    for i = 1:num_obs
        obs_vect(i).A = update_obstacle(obs_vect(i).A, t, dt);
        plot_obstacle(obs_vect(i).A);
    end
    
    if (gt_re)
        [X, dX, A] = generate_trajectories(x_state(0:3), xf, 1, T-t);
        S.A = A;
        time_offset = t; 
    end
    % Plot desired trajectory
    plot(X(1,:), X(2,:), '-r')
    % Update and Plot Car
    dxa = car_ode_bs(t, x_state, S);
    x_state = x_state + dxa*dt;
    plot_car(x_state, car_len, car_width);
    gif
end

%% PLAY GIF
web('test.gif')



