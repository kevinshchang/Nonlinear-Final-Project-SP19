%% Main for "Dynamic" Obstacle Avoidance

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

% Choice of generation/tracking method:
%   1. Differential Flatness
%   2. Feedback Linearization
%   3. Backstepping
%   4. Velocity Obstacle 
gt_type = 1;
gt_re = 0; % boolean for resampling of trajectory generation per timestep

% Car Parameters:
l = car_width;

% Boundary Conditions:
x0 = [car_len/2; lane_dist/2; 0]; % starting position
xf = [end_dist; lane_num*lane_dist; exit_ang]; % desired end position
T = sim_time;

%% First Plot

% Generating Obstacles:
for i = 1:num_obs
    y_rand = randi([1 3]);
    x_rand = randi([2 floor(end_dist/car_len)]);
    obs_vect(i).A = generate_obstacle([(car_len*x_rand) - (car_len/2),(lane_dist*y_rand) - (lane_dist/2)], car_len, car_width, obs_vel);
end

% Generating Environment:
f = figure();
% Our car
rectangle('Position', [0 (lane_dist/2 - car_width/2) car_len car_width], 'Curvature', 0.2, 'FaceColor', 'b', 'EdgeColor', 'b');
hold on
% Adding lanes
for i = 1:lane_num-1
    yline(lane_dist*i, 'k-.');
end
% Adding obstacles
for i = 1:num_obs
    plot_obstacle(obs_vect(i).A);
end
% Desired end position
plot(xf(1), xf(2), '*');
axis([0 end_dist 0 lane_num*lane_dist]);

%% Generate and Plot Trajectory
[X, dX, A] = generate_trajectories(x0, xf, 1, T);
plot(X(1,:), X(2,:), '-r')
hold off

%% GIF?
gif('test.gif')

%%
for t=1:1000
    cla 
    % Update and Plot Obstacles
    for i = 1:num_obs
        obs_vect(i).A = update_obstacle(obs_vect(i).A, i, 0.2);
        plot_obstacle(obs_vect(i).A);
    end
    plot(X(1,:), X(2,:), '-r')
    % Our car
    rectangle('Position', [0 (lane_dist/2 - car_width/2) car_len car_width], 'Curvature', 0.2, 'FaceColor', 'b', 'EdgeColor', 'b');
    hold on
    % Adding lanes
    for i = 1:lane_num-1
        yline(lane_dist*i, 'k-.');
    end
    plot(xf(1), xf(2), '*');
    axis([0 end_dist 0 lane_num*lane_dist]);
    gif
end

%% PLAY GIF
web('test.gif')



