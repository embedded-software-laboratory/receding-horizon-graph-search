function [u, y_pred, info] = graph_search(scenario, iter)
% GRAPH_SEARCH  Expand search tree beginning at current node for Hp steps.

    info = struct;
    shapes_tmp = cell(scenario.nVeh,0);
    % Create tree with root node
    x = iter.x0(:,1);
    y = iter.x0(:,2);
    yaw = iter.x0(:,3);
    trim = iter.trim_indices;
    k = 0;
    g = 0;
    h = 0;
    info.tree = Tree(x,y,yaw,trim,k,g,h);
    
    % Array storing ids of nodes that may be expanded
    open_nodes = 1;
    open_values = sum_values(info.tree, open_nodes);
        
    % Expand leaves of tree until depth or target is reached or until there 
    % are no leaves
    while true
        % Choose cheapest node for expansion
        if numel(open_nodes) == 0
            ME = MException( ...
                'MATLAB:graph_search:tree_exhausted' ...
                ,'No more open nodes to explore' ...
            );
            throw(ME);
        end
        
        cur_node_id = open_nodes(1);
        
        % remove parent node
        open_nodes(1) = [];
        open_values(1) = [];

        % Eval edge 
        [is_valid, shapes] = eval_edge_exact(scenario, info.tree, cur_node_id);
        if ~is_valid
            % could remove node from tree here
            continue
        end
        shapes_tmp(:,cur_node_id) = shapes;
        if info.tree.k(cur_node_id) == scenario.Hp
            y_pred = return_path_to(cur_node_id, info.tree, scenario.mpa);
            u = zeros(scenario.nVeh);
            info.shapes = return_path_area(shapes_tmp, info.tree, cur_node_id);
            info.tree_path = fliplr(path_to_root(info.tree, cur_node_id));
            info.trim_indices = info.tree.trim(:,info.tree_path(2));
            info.open_nodes = open_nodes;
            info.open_values = open_values;
            break
        else
            % Expand chosen node
            new_open_nodes = expand_node(...
                scenario...
                ,iter...
                ,cur_node_id...
                ,info...
            );
            % add child nodes
            open_nodes = [open_nodes, new_open_nodes]; %#ok<AGROW>
            open_values = [open_values, sum_values(info.tree, new_open_nodes)]; %#ok<AGROW>
            [open_nodes, open_values] = sort_open_list(open_nodes, open_values);

            % % plot exploration
            % info.plot = visualize_exploration(scenario, info.tree, info.plot);
        end
    end
end
