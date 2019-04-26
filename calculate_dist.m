function [d,theta] = calculate_dist(p, A)
%p is position (x, y)
%A is obstacle

px = p(1);
py = p(2);

Ax = A.p(1);
Ay = A.p(2);
width = A.width;
height = A.height;

dx = max(abs(px - Ax) - width / 2, 0);
dy = max(abs(py - Ay) - height / 2, 0);

d =  dx * dx + dy * dy;
theta = atan2(dy,dx);