function uout = car_ctrl_vo(t, xa, u_star, S)
% Add velocity obstacle code for this
n = 50; % number of samples
tlim = 3.5; % sample time limit
minimum = 1000000; %abritary large number;

uout = [-100; -100];
dt = 0.05;



% first check if u_star works
if (u_star(1) > S.lastu(1)) 
    if (u_star(1) - S.lastu(1) > S.u_range(1)/2)
        u_star(1) = S.lastu(1) + S.u_range(1)/2;
    end
elseif (u_star(1) < S.lastu(1))
    if (S.lastu(1) - u_star(1) > S.u_range(1)/2)
        u_star(1) = S.lastu(1) - S.u_range(1)/2;
    end
end
if (u_star(2) > S.lastu(2)) 
    if (u_star(2) - S.lastu(2) > S.u_range(2)/2)
        u_star(2) = S.lastu(2) + S.u_range(2)/2;
    end
elseif (u_star(2) < S.lastu(2))
    if (S.lastu(2) - u_star(2) > S.u_range(2)/2)
        u_star(2) = S.lastu(2) - S.u_range(2)/2;
    end
end


u = u_star; %sample from control set 
free =1;
for j= 1:S.num_obs
    A = S.obs_vect(j).A;
    pB = A.p - xa(1:2); %set car as origin of obstacle distance
    vB = [A.v;0];
    C = [0; 0]; % position of car relative to itself
    for k=0:dt:tlim
        B = pB + vB*k; %position of obstacle relative to car (orientation is still global frame)
        d = C - B;
        % I set 1.5 meter as collision distance cause y not
        if (norm(d) < 1.5)
            free = 0;
        end
        if (~free)
            break
        end
        %We update our position of a car by assuming controls are constant, and integrating.
        %We did not integrate x and y because those gave us singularities
        %with respect to u2;
        [dx,dy] = predict(u, k, xa, S);
        C= C+[dx;dy]*dt;
    end 
end
if (free)
    uout = u;
    return;
end

for i = 1:n
    free = 1;
    u = sample(S); %sample from control set 
    for j= 1:S.num_obs
        A = S.obs_vect(j).A;
        pB = A.p - xa(1:2); %set car as origin of obstacle distance
        vB = [A.v;0];
        C = [0;0]; %position of car relative to itself; 
        for k=0:dt:tlim
            B = pB + vB*k; %position of obstacle 
            d = C - B;
            % I set 1.5 meter as collision distance cause y not
            if (norm(d) < 1.5)
                free = 0;
            end
            if (~free)
                break
            end
            %We update our position of a car by assuming controls are constant, and integrating.
            %We did not integrate x and y because those gave us singularities
            %with respect to u2;
            [dx,dy] = predict(u, k, xa, S);
            C= C+[dx;dy]*dt;
        end 
        if (~free)
            break
        end
    end
    if (free)
%         u(2) = mod(u(2),2*pi);
%         u_star(2) = mod(u_star(2),2*pi);
        u_norm = norm(u-u_star);
        if (u_norm<minimum)
            minimum = u_norm;
            uout = u;
        end
    end
end
if (uout(1) == -100)
    warning('COULD NOT FIND VALID INPUT. INPUT IS U*');
    uout = [0; 0];
end


