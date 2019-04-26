function new_A = update_obstacle(A, t, dt)
new_A = A;
new_A.p(1) = A.p(1) + A.v*dt;