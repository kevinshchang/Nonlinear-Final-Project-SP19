function ua = car_ctrl_fl(t, xa, S)
% tracking control law

% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);
d3yd = S.A*d3poly3(t);

% get current output
y = car_h(xa);

% compensator, i.e.  xi=u1,u2
xi = xa(end-1:end);
u1 = xi(1);
u2 = xi(2);

V = xa(4);
R = rot(xa(3));

% current velocity
dy = [cos(xa(3)); sin(xa(3))]*V;
d2y = R*[u2; ((V^2)*tan(u1))/S.l];

% error state
z1 = y - yd;
z2 = dy - dyd;
z3 = d2y - d2yd;

% virtual inputs
v = d3yd - S.k(1)*z1 -S.k(2)*z2 - S.k(3)*z3;
% augmented inputs
ua = (R*[1 0;0 V^2/((cos(u1)^2)*S.l)])\(v - R*[-((V^3)*(tan(u1)^2))/(S.l^2)  
    (V/S.l)*tan(u1)*u2 + (2*V*u2*tan(u1))/S.l]);
ua = flip(ua);
