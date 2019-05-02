function dxa = car_ode_fl(t, xa, S)
% car ODE with feedback linearization
ua = car_ctrl(t, xa, S);

xi = xa(end);
u1 = ua(1);
u2 = ua(2);


dxa = [cos(xa(3))*xi;
      sin(xa(3))*xi;
      (tan(u1)*xi)/S.l;
      u2];