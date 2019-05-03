function f = plot_car(x_state, S)
%https://math.stackexchange.com/questions/2518607/how-to-find-vertices-of-a-rectangle-when-center-coordinates-and-angle-of-tilt-is
%http://msl.cs.uiuc.edu/planning/node658.html
    theta = x_state(3);
    w = S.car_len/2;
    b = S.car_width/2;
    xc = x_state(1) + (S.l/2)*cos(theta);
    yc = x_state(2) + (S.l/2)*sin(theta);
    x1 = xc - w*cos(theta) - b*sin(theta);
    x2 = xc + w*cos(theta) - b*sin(theta);
    x3 = xc + w*cos(theta) + b*sin(theta);
    x4 = xc - w*cos(theta) + b*sin(theta);
    
    y1 = yc - w*sin(theta) + b*cos(theta);
    y2 = yc + w*sin(theta) + b*cos(theta);
    y3 = yc + w*sin(theta) - b*cos(theta);
    y4 = yc - w*sin(theta) - b*cos(theta);
    
    xdata = [x1 x2 x3 x4];
    ydata = [y1 y2 y3 y4];
    h = patch('XData',xdata,'YData',ydata,'FaceColor','blue');
    
