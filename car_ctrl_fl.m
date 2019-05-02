function ua = car_ctrl_fl(t, xa, S)
% tracking control law

% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% get current output
y = car_h(xa);

% compensator, i.e.  xi=u1,u2
xi = xa(end);

% current velocity
dy = [cos(xa(3)); sin(xa(3))]*xi;

% error state
z1 = y - yd;
z2 = dy - dyd;

% virtual inputs
v = d2yd - S.k(1)*z1 -S.k(2)*z2;
% augmented inputs ua=(du2, du1) (FLIPPED COMPARED TO AUGMENTED STATE)
A = [-xa(4)^2*sin(xa(3)) cos(xa(3));
    xa(4)^2*cos(xa(3)) sin(xa(3))];
ua = inv(A)*v;
ua(1) = atan(ua(1));
