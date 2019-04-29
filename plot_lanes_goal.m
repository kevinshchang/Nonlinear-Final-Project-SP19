function f = plot_lanes_goal(lane_num, lane_dist, xf, end_dist)
 % Plot lanes and goal
    for i = 1:lane_num-1
        yline(lane_dist*i, 'k-.');
    end
    plot(xf(1), xf(2), '*');
    axis([0 end_dist 0 lane_num*lane_dist]);