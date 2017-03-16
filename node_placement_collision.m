function node_loc = node_placement_collision(loc_init, loc_fin, obstacle,R)
    max_dist = 0.8;
    eps = 0.01;
    dir = (loc_fin - loc_init)/norm(loc_fin - loc_init);
    dist = norm(loc_fin - loc_init);
    distance = min(max_dist, dist);
    for i = eps:eps:distance
        pos = loc_init + dir*i;
        pos = transpose(pos);
        col = checkCollision(pos, R, obstacle);
        if (col == 1)
            index = i - eps;
            node_loc = loc_init + dir*index;
            return;
        end
    end
    node_loc = loc_init+dir*distance;
end