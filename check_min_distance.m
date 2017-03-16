function [min_node, min_distance] = check_min_distance(location_graph, new_node)
    [~,N] = size(location_graph);
    distances = Inf*zeros(N,1);
    for i = 1:N
       distances(i) = norm(new_node - location_graph(:,i));
    end
    [min_distance, min_node] = min(distances);
end