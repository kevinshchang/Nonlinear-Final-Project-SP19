function [dx, dy] = predict(u, t, xa, S)

if (S.gt_type == 1)
    dx = u(1)*cos(u(1)*tan(u(2))*t/S.l + xa(3));
    dy = u(1)*sin(u(1)*tan(u(2))*t/S.l + xa(3));
else
    v = u(1)*t+ xa(4);
    theta = (u(1)*t/2 + xa(4))*t*tan(u(2))/S.l + xa(3);
    dx = v*cos(theta);
    dy = v*sin(theta);
end