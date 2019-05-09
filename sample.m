function u = sample(S)
% randomly sample a control input from the control set
lastu1 = S.lastu(1);
lastu2 = S.lastu(2);
u1_range = S.u_range(1);
u2_range = S.u_range(2);
newu1 = lastu1 + (rand*u1_range - u1_range/2);
newu2 = lastu2 + (rand*u2_range - u2_range/2);

u = [newu1;newu2];
