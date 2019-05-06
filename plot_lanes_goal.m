function f = plot_lanes_goal(S)
 % Plot lanes and goal
    for i = 1:S.lane_num-1
        yline(S.lane_dist*i, 'k-.');
    end
    plot(S.xf(1), S.xf(2), '*');