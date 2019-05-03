function ua = car_ctrl_care(t, xa, S)
% tracking control law (differential flatness)

% get desired outputs:
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% desired angle
x3d = atan2(dyd(2), dyd(1));
x4d = norm(dyd);
xd = [yd; x3d; x4d];

% desired inputs
ud = [atan2(S.l*(d2yd(2)*dyd(1) - d2yd(1)*dyd(2)),x4d^3);
    ((dyd(1)*d2yd(1)+dyd(2)*d2yd(2)))/x4d];
    

% linearization, i.e. \dot (x - xd) = A(x - xd) + B(u - ud)
A = [0 0 -sin(x3d)*x4d cos(x3d);
     0 0 cos(x3d)*x4d sin(x3d);
     0 0 0 tan(ud(1))/S.l;
     0 0 0 0];

B = [0 0;
     0 0;
     (x4d*(sec(ud(1))^2))/S.l 0;
     0 1];

% compute gain matrix for linear system (this solves the Lyapunov
% equation for the linearization dynamics)
[X,L,K] = care(A, B, eye(4));
e = xa - xd;
ua = ud - 5*K*e;
