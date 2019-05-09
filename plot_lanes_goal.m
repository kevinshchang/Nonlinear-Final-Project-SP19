function f = plot_lanes_goal(S)
 % Plot lanes and goal
    for i = 1:S.lane_num
        yline(S.lane_dist*i, 'k-.');
    end
    yline(0, 'k');
    yline((S.lane_num+1)*S.lane_dist, 'k');
    plot(S.xf(1), S.xf(2), '*');