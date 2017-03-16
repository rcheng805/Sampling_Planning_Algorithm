clear all
close all

init_pos = [1; 1];
R = 0.4;
obs{1} = [2, 5; 3, 6; 2, 4];
% obs{2} = [6, 1; -3, 4; 0, 4];
% obs{3} = [6, 1; 6, 0; 12, 0; 12, 1];
% obs{1} = [1, 1; 1, 6; 2, 1];
% obs{2} = [3, 0; 6, -1; 7, 2; 3, 3; 2, 1.5];

goal_pos = [4;4];
%goal_pos = [-1; 3];

%Plot initial configuration
figure(3)
for i = 1:length(obs)
    obs_plot = [obs{i}; obs{i}(1,:)];
    plot(obs_plot(:,1),obs_plot(:,2))
    hold on
end
viscircles(transpose(init_pos),R)
viscircles(transpose(goal_pos),R)
scatter(goal_pos(1), goal_pos(2), 'r')
hold off
title('Initial Configuration')
set(gca,'FontSize',14)

%Initialize graph
G = digraph;
location_graph = zeros(2,1000);
connection_graph = zeros(3,1000);
G = addnode(G,1);
location_graph(:,1) = init_pos;
%Continually add new points to the graph and connect them to the closest
%exiting poinst in the graph
i = 1;
while true
    i = i + 1;
    %Sample points, and sample goal point with 10% probability
    if (rand <= 0.8)
        new_node = sample_points(location_graph(:,randi(i)), goal_pos,1,pi/6);
        if (checkCollision(transpose(new_node), 0.05, obs))
            i = i - 1;
            continue
        end
    else
        new_node = goal_pos;
    end
    
    %Check for the nearest node in the graph
    for j = 1:i-1
        [nearest_node, ~] = check_min_distance(location_graph(:,1:i-1), new_node);
    end
    
    %Place new node based on obstacles
    loc_init = location_graph(:,nearest_node);
    loc_fin = new_node;
    new_nod_loc = node_placement_collision(loc_init, loc_fin, obs,R);
    if (checkCollision(transpose(new_nod_loc), R, obs) == 1)
        i = i - 1;
        continue
    end
    
    min_distance = norm(new_nod_loc - loc_init);
    
    %Add new node and edge to the graph
    location_graph(:,i) = new_nod_loc;
    connection_graph(:,i) = [nearest_node, i, min_distance]; 
    G = addedge(G,nearest_node, i,min_distance);
    if (sum(new_nod_loc == goal_pos) == 2)
        'Goal Reached'
        iterations = i;
        break;
    end
end

%Plot searched paths

figure(1)
for i = 1:length(obs)
    obs_plot = [obs{i}; obs{i}(1,:)];
    plot(obs_plot(:,1),obs_plot(:,2))
    hold on
end
viscircles(transpose(init_pos),R)
viscircles(transpose(goal_pos),R)
scatter(goal_pos(1), goal_pos(2), 'r')
for i = 2:iterations
    line_pos = connection_graph(:,i);
    node1 = line_pos(1);
    node2 = line_pos(2);
    dist = line_pos(3);
    plot([location_graph(1,node1), location_graph(1,node2)], ...
        [location_graph(2,node1), location_graph(2,node2)]);
end 
hold off
title('Paths Explored under Sampling Algorithm')
set(gca,'FontSize',14)


%Search for path from initial node to final node
closed = [];
closed_index = 1;
visited = zeros(iterations,1);
visited(1) = 1;
open = [1; distances(G,1,iterations)];
ind = 1;
while (true)
    ind = ind + 1;
    [~,V_best_ind] = min(open(2,:));
    V_best = open(1,V_best_ind);
    if (V_best == 1)
        closed = [1;1];
    else
        closed = [closed, [V_best; predecessors(G,V_best)]];
    end
    open(:,V_best_ind) = [];
    if (V_best == iterations)
        closed
        'solution found'
        break;
    end
    nodes = successors(G,V_best);
    cost_local = Inf*zeros(length(nodes),1);
    for i = 1:length(nodes)
        if (visited(nodes(i)) == 0)
            visited(nodes(i)) = 1;
            open = [open, [nodes(i); distances(G,nodes(i),iterations)]];
        else
            if (distances(G,nodes(i),1) > distances(G,V_best,1) + ...
                    norm(location_graph(:,V_best) - location_graph(:,nodes(i))))
                I = find(closed(1,:)==nodes(i));
                closed(2,I) = V_best;
                G = addedge(G,V_best, nodes(i));
            end
        end
    end
end


%Plot searched paths
figure(2)
for i = 1:length(obs)
    obs_plot = [obs{i}; obs{i}(1,:)];
    plot(obs_plot(:,1),obs_plot(:,2))
    hold on
end

viscircles(transpose(init_pos),R)
viscircles(transpose(goal_pos),R)
scatter(goal_pos(1), goal_pos(2), 'r')
for i = 2:iterations
    line_pos = connection_graph(:,i);
    node1 = line_pos(1);
    node2 = line_pos(2);
    dist = line_pos(3);
%     plot([location_graph(1,node1), location_graph(1,node2)], ...
%         [location_graph(2,node1), location_graph(2,node2)]);
end 
loc = location_graph(:,closed(1,:));
plot(loc(1,:),loc(2,:),'k')
hold off
title('Chosen Path')


