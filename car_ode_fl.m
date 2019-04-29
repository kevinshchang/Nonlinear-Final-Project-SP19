function dxa = car_ode_fl(t, xa, S)
% car ODE with feedback linearization
ua = car_ctrl(t, xa, S);

u1 = xa(end-1);
u2 = xa(end);
du2 = ua(1);
du1 = ua(2);


dxa = [cos(xa(3))*xa(4);
      sin(xa(3))*xa(4);
      xa(4)*tan(u1)/S.l;
      u2;
      du1;
      du2];