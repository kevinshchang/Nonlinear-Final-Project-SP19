function [ua, S] = car_ctrl_bs_switch(t, xa, S)
% tracking control law

t_rel = t - S.t_offset;

% get desired outputs:
yd = S.A*poly3(t_rel);
dyd = S.A*dpoly3(t_rel);
d2yd = S.A*d2poly3(t_rel);

% if (norm(xa(1:2) - yd) > 3)
%     S.t_offset = t;
%     [S.X, S.A] = generate_trajectories_simple(xa, S.xf, 10, S.T-t);
%     plot(S.X(1,:), S.X(2,:), '-b')
%     yd = S.A*poly3(0);
%     dyd = S.A*dpoly3(0);
%     d2yd = S.A*d2poly3(0);
% end


% get current output
y = car_h(xa);

% current velocity
dy = [cos(xa(3)); sin(xa(3))]*xa(4);

% errors
e = y - yd;
de = dy - dyd;

% z-state
z = -dyd + S.k(1)*e + dy;

s = rot(xa(3)).'*(d2yd - S.k(1)*de - S.k(2)*z);

% augmented inputs ua=(du1, u2)
ua = [s(1);
      atan2(s(2)*S.l,xa(4)^2)];