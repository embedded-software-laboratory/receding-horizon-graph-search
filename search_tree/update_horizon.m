function [leaf_nodes, candidates, search_tree, max_id] = update_horizon(cur_node, next_node, leaf_nodes, candidates, search_tree, next_id, obstacles, motion_graph, situation_costs, init_poses, target_poses, visited_nodes, max_id, is_goals)
        
    n_veh = length(motion_graph.motionGraphList);
    visited = true;
  
    for i = 1 : n_veh
            
        if is_goals(i)
            continue
        end

        maneuver = motion_graph.motionGraphList(i).maneuvers{cur_node.trims(i), next_node.trims(i)};

        next_node.depth = cur_node.depth + 1;
        next_node.yaws(i) = cur_node.yaws(i) + maneuver.dyaw;
        [next_node.xs(i), next_node.ys(i)] = translate_global(cur_node.yaws(i), cur_node.xs(i), cur_node.ys(i), maneuver.dx, maneuver.dy);

        [shape_x, shape_y] = translate_global(cur_node.yaws(i), cur_node.xs(i), cur_node.ys(i), maneuver.area(1,:), maneuver.area(2,:));
        next_node.shapes(i) = polyshape(shape_x,shape_y,'Simplify',false);
        
        % Abort update if collision is detected
        if collision_with(i, next_node.shapes, obstacles)
            return
        end
        
        % Check if similar state was visited until difference occurs
        % if visited
        %     visited = has_visited(i, next_node, search_tree, visited_nodes, 0.1);
        % end
    end
    
    % Similar state was already explored
    %if visited
    %    return
    %end

    [next_node.g_values, next_node.h_values] = calculate_next_values_reference(cur_node.g_values, situation_costs, init_poses, target_poses, next_node, motion_graph);
    next_node.id = max_id + 1;
    [search_tree, max_id] = search_tree.addnode(next_id, next_node);
    
    is_goals = is_goal(next_node, target_poses);
    if next_node.depth < h_p && ~all(is_goals)
        leaf_nodes = [leaf_nodes, max_id];
    else
        candidates = [candidates, max_id];
    end

end


