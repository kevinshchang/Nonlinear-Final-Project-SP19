function u = car_ctrl_care_simple(t, x, S)
% tracking control law
% get desired outputs (x-y positions, velocities, accelerations)
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% desired angle
x3d = atan2(dyd(2), dyd(1));

xd = [yd; x3d];

% desired inputs
ud = [norm(dyd); 
      atan2(S.l*(d2yd(2)*dyd(1) - d2yd(1)*dyd(2)),norm(dyd)^3)];

% linearization, i.e. \dot (x - xd) = A(x - xd) + B(u - ud)
A = [0 0 -sin(x3d);
     0 0 cos(x3d);
     0 0 0]*ud(1);

B = [cos(x3d) 0;
     sin(x3d) 0;
     tan(ud(2))/S.l (ud(1)*(sec(ud(2))^2))/S.l];

% compute gain matrix for linear system (this solves the Lyapunov
% equation for the linearization dynamics)
[X,L,K] = care(A, B, eye(3));

% set control law
u = ud - K*(x - xd);