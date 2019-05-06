function uout = car_ctrl_vo(t, xa, u_star, S)
% Add velocity obstacle code for this
n = 50; % number of samples
tlim = 3.5; % sample time limit
minimum = 1000000; %abritary large number;

uout = [-100; -100];
for i = 1:n
    free = 1;
    u = sample(S); %sample from control set 
    for j= 1:S.num_obs
        A = S.obs_vect(j).A;
        pB = A.p - xa(1:2); %set car as origin of obstacle distance
        vB = [A.v;0];
        for k=0:0.05:tlim
            C = [(sin(u(1)*tan(u(2))*k))/tan(u(2)); 
                -(cos(u(1)*tan(u(2))*k)/tan(u(2))) + 1/tan(u(2))]; % position of the car
            B = pB + vB*k;
            d = C - B;
            % I set 1 meter as collision distance cause y not
            if (norm(d) < 1.5)
                free = 0;
            end
        end 
    end
    if (free)
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
