function points = sample_points(init_pos, final_pos,N,sigma)
    points = zeros(2,N);
    increment = 10*rand;
    dir = final_pos - init_pos;
    theta = atan2(dir(2), dir(1));
    for i = 1:N
        angle = normrnd(theta, sigma);
        points(:,i) = [init_pos(1) + increment*cos(angle), ...
            init_pos(2) + increment*sin(angle)];
    end
end