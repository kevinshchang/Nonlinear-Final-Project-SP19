function [dxa, S, us] = car_ode_vo(t, dt, xa, S, us)
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
gif


% Setup of initial conditions
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
    [ua, S] = car_ctrl_bs_simple(t,xa,S);
    ua = car_ctrl_vo(t, xa, ua, S);
    u1 = ua(1);
    u2 = ua(2);
    dxa = [cos(xa(3))*xa(4);
      sin(xa(3))*xa(4);
      (tan(u2)*xa(4))/S.l;
      u1];

elseif (S.gt_type == 4)
end
    


% ua(2) = mod(ua(2), 2*(pi));
S.lastu = ua;
us = [us, ua];



