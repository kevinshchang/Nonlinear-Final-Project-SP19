function A = generate_obstacle(p, width, height, v)
%p starting coordinates (x,y) 
%width is x size (centered around p)
%height is y size (centered around p)
%v(t) is velocity function

A.p = p;
A.width = width;
A.height = height;
A.v = v;