function f = plot_obstacle(A)
%plots an obstacle
rectangle('Position',[A.p(1)-(A.width/2) A.p(2)-(A.height/2) A.width A.height], 'FaceColor', 'r', 'EdgeColor', 'r');
