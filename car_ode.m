function dx = car_ode(t, x, S)
% Allows for internal customization of ODEs

% Steering force addition
% if ~isempty(S.po) 
%     g = q - S.po;
%     d = norm(g) - S.ro;
%     g = d*g/norm(g);
%     a = -q_dot'*g/(norm(q_dot)*d);
%     if (d < S.do && abs(a) < pi/2)
%         u = u + (S.k(3)/d*[0 -1 0; 1 0 0; 0 0 0]*q_dot);
%     end
% end
cla 
hold on
% Plot planes and goal
plot_lanes_goal(S);

% Update and Plot Obstacles
for i = 1:S.num_obs
    S.obs_vect(i).A = update_obstacle(S.obs_vect(i).A, t);
end
plot_obstacle(S);

% if (S.gt_re)
%     [X, A] = generate_trajectories(x, xf, T-t);
%     S.A = A; 
%     S.X = X;
% end
% Plot desired trajectory
plot(S.X(1,:), S.X(2,:), '-r')
% Update and Plot Car
plot_car(x, S);
gif


if (S.gt_type == 1)
    u = car_ctrl_care(t,x,S);
elseif (S.gt_type == 2)
    u = car_ctrl_fl(t,x,S);
elseif (S.gt_type == 3)
    u = car_ctrl_bs(t,x,S);
elseif (S.gt_type == 4)
    u = car_ctrl_vo(t,x,S);
end
if (S.gt_type ~= 2)
    u1 = u(1);
    u2 = u(2);
    dx = [cos(x(3))*x(4);
          sin(x(3))*x(4);
          x(4)*tan(u1)/S.l;
          u2];
else
    u1 = x(end-1);
    u2 = x(end);
    du1 = u(1);
    du2 = u(2);
    dx = [cos(x(3))*x(4);
      sin(x(3))*x(4);
      x(4)*tan(u1)/S.l;
      u2;
      du1;
      du2];
end