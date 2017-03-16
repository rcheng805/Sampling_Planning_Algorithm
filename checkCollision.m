function col = checkCollision(pos, R, obstacle)
    num_obs = length(obstacle);
    for k = 1:num_obs
        [in, ~] = inpolygon(pos(1),pos(2),obstacle{k}(:,1),obstacle{k}(:,2));
        if (in == 1)
            col = 1;
            return
        end
        N = length(obstacle{k});
        dist = zeros(N);
        for i = 1:N-1
            edge1 = obstacle{k}(i+1,:) - obstacle{k}(i,:);
            edge2 = pos - obstacle{k}(i+1,:);
            edge3 = pos - obstacle{k}(i,:);
            if(dot(-edge1,edge2)*dot(edge1,edge3)>=0)
                points = [obstacle{k}(i,:), 1; obstacle{k}(i+1,:),1; pos, 1];
                dist(i) = abs(det(points))/norm(edge1);
            else
                dist(i) = min(norm(edge2), norm(edge3));
            end
        end
        edge1 = obstacle{k}(1,:) - obstacle{k}(N,:);
        edge2 = pos - obstacle{k}(1,:);
        edge3 = pos - obstacle{k}(N,:);
        if(dot(-edge1,edge2)*dot(edge1,edge3)>=0)
            points = [obstacle{k}(N,:), 1; obstacle{k}(1,:),1; pos, 1];
            dist(N) = abs(det(points))/norm(edge1);
        else
            dist(N) = min(norm(edge2), norm(edge3));
        end

        min_dist = min(dist);
        if (min_dist < R)
            col = 1;
            return;
        else
            col = 0;
        end
        
        for i = 1:N
            if (norm(pos - obstacle{k}(i,:)) < R)
                col = 1;
                return;
            end
        end
    end
    
end