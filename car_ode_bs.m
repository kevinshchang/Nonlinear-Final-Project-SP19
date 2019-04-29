function dxa = car_ode_bs(t, x, S)
% basic car ODE with backstepping control

u = car_ctrl_bs(t ,x, S);

u1 = u(1);
u2 = u(2);

dxa = [cos(x(3))*x(4);
      sin(x(3))*x(4);
      x(4)*tan(u1)/S.l;
      u2];