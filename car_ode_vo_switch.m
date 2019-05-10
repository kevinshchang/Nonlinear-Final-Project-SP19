function [dxa, S, us] = car_ode_vo_switch(t, dt, xa, S, us)
cla 
hold on


% Plot planes and goal
plot_lanes_goal(S);
% Update and Plot Obstacles
for i = 1:S.num_obs
    S.obs_vect(i).A = update_obstacle(S.obs_vect(i).A, dt);
    if (norm(S.obs_vect(i).A.p - xa(1:2)) < 1.25) 
        error('collision')
    end
end
plot_obstacle(S);
% Plot desired trajectory
plot(S.X(1,:), S.X(2,:), '-r')
% Plot Car
plot_car(xa, S);

if (switchable(t, xa, S) == 1)
    S.switching = 1;
    S.switch_count = S.switch_count + 1;
    S.new_goal = [xa(1)+15; (S.switch_count+0.5)*S.lane_dist; 0];
    [S.X, S.A] = generate_trajectories_simple(xa, S.new_goal, 10, 2.5);
    S.t_offset = t;
end

if (S.switching == 1)
    if (S.gt_type == 1)
        [ua, S] = car_ctrl_care_simple(t,xa,S);
        u1 = ua(1);
        u2 = ua(2);
        dxa = [cos(xa(3))*u1;
          sin(xa(3))*u1;
          (tan(u2)*u1)/S.l];
    elseif (S.gt_type == 3)
        [ua, S] = car_ctrl_bs_switch(t,xa,S);
        ua = car_ctrl_vo(t, xa, ua, S);
        u1 = ua(1);
        u2 = ua(2);
        dxa = [cos(xa(3))*xa(4);
          sin(xa(3))*xa(4);
          (tan(u2)*xa(4))/S.l;
          u1];
    end
    if (norm(xa(1:2) - S.new_goal(1:2)) < 0.5)
        S.switching = 0;
        S.new_goal = S.xm(S.switch_count).m;
        [S.X, S.A] = generate_trajectories_simple(xa, S.new_goal, 10, S.T-t);
    end
else
    if (S.gt_type == 1)
        [ua, S] = car_ctrl_care_simple(t,xa,S);
        ua = car_ctrl_vo(t, xa, ua, S);
        u1 = ua(1);
        u2 = ua(2);
        dxa = [cos(xa(3))*u1;
          sin(xa(3))*u1;
          (tan(u2)*u1)/S.l];
    elseif (S.gt_type == 2)

    elseif (S.gt_type == 3)
        [ua, S] = car_ctrl_bs_switch(t,xa,S);
        ua = car_ctrl_vo(t, xa, ua, S);
        u1 = ua(1);
        u2 = ua(2);
        dxa = [cos(xa(3))*xa(4);
          sin(xa(3))*xa(4);
          (tan(u2)*xa(4))/S.l;
          u1];

    elseif (S.gt_type == 4)
    end
end
gif

% ua(2) = mod(ua(2), 2*(pi));
S.lastu = ua;
us = [us, ua];



