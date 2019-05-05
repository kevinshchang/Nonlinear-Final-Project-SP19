function dxa = car_ode_vo(t, xa, S)
% car ODE with velocity obstacle
ua = car_ctrl_vo(t, xa, S);
S.lastu = ua;
u1 = ua(1);
u2 = ua(2);


dxa = [cos(xa(3))*u1;
      sin(xa(3))*u1;
      (tan(u2)*u1)/S.l];