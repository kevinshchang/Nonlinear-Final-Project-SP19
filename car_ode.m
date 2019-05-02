function dx = car_ode(t, x, S)
% Allows for internal customization of ODEs

if (S.gt_type == 1)
    u = car_ctrl_care(t,x,S);
elseif (S.gt_type == 2)
    u = car_ctrl_fl(t,x,S);
elseif (S.gt_type == 3)
    u = car_ctrl_bs(t,x,S);
elseif (S.gt_type == 4)
    u = car_ctrl_vo(t,x,S);
end

% Steering force addition
% if ~isempty(S.po) 
%     g = q - S.po;
%     d = norm(g) - S.ro;
%     g = d*g/norm(g);
%     a = -q_dot'*g/(norm(q_dot)*d);
%     if (d < S.do && abs(a) < pi/2)
%         u = u + (S.k(3)/d*[0 -1 0; 1 0 0; 0 0 0]*q_dot);
%     end
% end

u1 = u(1);
u2 = u(2);

if (S.gt_type == 1) 
    dx = [cos(x(3))*u1;
       sin(x(3))*u1;
       u1*(tan(u2))/S.l];
elseif (S.gt_type == 2 || S.gt_type == 3)
    dx = [cos(x(3))*x(4);
          sin(x(3))*x(4);
          x(4)*tan(u1)/S.l;
          u2];
end