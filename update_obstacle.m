function A_new = update_obstacle(A, dt)
    A_new = A;
    %percent chance velocity changes per second
    chance_change = 0.2;
    chance_change_perdt = chance_change*dt;
    if (rand < chance_change_perdt) 
        %velocity changes by plus minus 1
        A_new.v = A_new.v + (A_new.v/norm(A_new.v))*((rand*5) - 2.5);
    end
    A_new.p = A_new.p + A_new.v*dt;

