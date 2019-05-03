function A_new = update_obstacle(A, t)
    A_new = A;
    A_new.p(1) = A_new.p0(1) + A_new.v*t;

