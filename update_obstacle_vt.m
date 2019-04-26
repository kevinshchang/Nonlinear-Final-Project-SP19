function new_A = update_obstacle_vt(A, t, dt)
new_A = A;
new_A.p(1) = A.p(1) + A.v(t)*dt;
