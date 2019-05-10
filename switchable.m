function b = switchable(t, xa, S)

b = 0;

if (S.switch_count >= S.lane_num)
    return;
end

if (S.switching == 1)
    return;
end

tlim = 2.5; % sample time limit
tlim_wall = 1; %sample time limit for walls
dt = 0.05;
free = 1;

new_goal = [xa(1)+15; (S.switch_count+1.5)*S.lane_dist; 0];
[X, A] = generate_trajectories_simple(xa, new_goal, 10, tlim);
plot(X(1,:), X(2,:), '-g')

for j= 1:S.num_obs
    O = S.obs_vect(j).A;
    pB = O.p - xa(1:2); %set car as origin of obstacle distance
    vB = O.v;
    C = [0;0]; %position of car relative to itself; 
    for k=0:dt:tlim
        B = pB + vB*k; %position of obstacle 
        d = C - B;
        %Check freeway walls
%         if (k<tlim_wall)
%             y = C(2) + xa(2);
%             if (y >= S.lane_dist*(S.lane_num+1) || y <= 0)
%                 free = 0;
%             end
%         end
        % I set 1.5 meter as collision distance cause y not
        if (norm(d) < 1)
            free = 0;
        end
        if (~free)
            break
        end
        %We update our position of a car by assuming controls are constant, and integrating.
        %We did not integrate x and y because those gave us singularities
        %with respect to u2;
        dxy = A*poly3(k);
        C= C+dxy*dt;
    end 
    if (~free)
        break
    end
end
if (free)
    b = 1;
end


