function dxa = car_ode_vo(t, xa, S)


cla 
hold on
% Plot planes and goal
plot_lanes_goal(S);

% Update and Plot Obstacles
for i = 1:S.num_obs
    S.obs_vect(i).A = update_obstacle(S.obs_vect(i).A, t);
end
plot_obstacle(S);

% Plot desired trajectory
plot(S.X(1,:), S.X(2,:), '-r')
% Update and Plot Car
plot_car(xa, S);
gif

ua = car_ctrl_care_simple(t,xa,S);
ua = car_ctrl_vo(t, xa, ua, S);
S.lastu = ua;
u1 = ua(1);
u2 = ua(2);


dxa = [cos(xa(3))*u1;
      sin(xa(3))*u1;
      (tan(u2)*u1)/S.l];