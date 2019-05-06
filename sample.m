function u = sample(S)
% randomly sample a control input from the control set
lastu1 = S.lastu(1);
lastu2 = S.lastu(2);

lastu1 = lastu1 + (rand*20 - 10);
lastu2 = lastu2 + (rand*.3 - .15);

u = [lastu1;lastu2];
