function ua = car_ctrl_bs(t, x, S)
% tracking control law (backstepping)
% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);
%d3yd = S.A*d3poly3(t);

% get current output
y = car_h(x);
v = x(end);
R = rot(x(3));

k0 = S.k(1);
k1 = S.k(2);

dy = [v*cos(x(3));
    v*sin(x(3))];
e = y-yd;
de = dy - dyd;
z = -yd + S.k(1)*e + dy;
s = R'*(d2yd - e - S.k(1)*de - S.k(2)*z);
u2 = s(1);
u1 = atan2(s(2)*S.l, v^2);
ua = [u1;u2];