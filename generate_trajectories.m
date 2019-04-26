function [X, dX, A] = generate_trajectories(x0, xf, u1, T)

% boundary conditions in flat output space 
y0 = car_h(x0);
yf = car_h(xf);
dy0 = u1*[cos(x0(3)); sin(x0(3))]; % desired starting velocity
dyf = u1*[cos(xf(3)); sin(xf(3))]; % desired end velocity

% compute path coefficients
A = poly3_coeff(y0, dy0, yf, dyf, T);

X = A*poly3([0:.01:T]);
dX = A*dpoly3([0:.01:T]);
