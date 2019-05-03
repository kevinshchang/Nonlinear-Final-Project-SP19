function f = plot_obstacle(S)
%plots an obstacle
for i = 1:S.num_obs
    A = S.obs_vect(i).A;
    rectangle('Position',[A.p(1)-(A.width/2) A.p(2)-(A.height/2) A.width A.height], 'FaceColor', 'r', 'EdgeColor', 'r');
end