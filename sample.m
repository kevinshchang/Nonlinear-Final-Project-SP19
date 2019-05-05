function u = sample(S)
% randomly sample a control input from the control set
lastu1 = S.lastu(1);
lastu2 = S.lastu(2);

lastu1 = lastu1 + (random()*10-5);
lastu2 = lastu2 + (random()*.25 -.125);

u = [lastu1;lastu2];
