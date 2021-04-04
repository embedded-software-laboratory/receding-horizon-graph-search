% checks whether the vehicles have reached their goal positions
% @result is boolean
function result = is_goal(cur_node, scenario)
    d = euclidean_distance(...
        cur_node.xs, ...
        cur_node.ys,...
        [scenario.vehicles(:).x_goal],...
        [scenario.vehicles(:).y_goal]...
    );
    result = (d < offset);
end

