function [u, y_pred, info, scenario] = pb_controller(scenario, iter)
% PB_CONTROLLER    Plan trajectory for one time step using a priority-based controller.
%     Controller simulates multiple distributed controllers.


switch scenario.priority_option
    case 'topo_priority'
        obj = topo_priority(scenario);
        [groups, directed_adjacency] = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'right_of_way_priority'
        obj = right_of_way_priority(scenario,iter);
        right_of_way = true;
        [veh_at_intersection, groups, edge_to_break, directed_adjacency] = obj.priority();  
    case 'constant_priority'
        obj = constant_priority(scenario);
        [groups, directed_adjacency] = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'random_priority' 
        obj = random_priority(scenario);
        [groups, directed_adjacency] = obj.priority(); 
        right_of_way = false;
        veh_at_intersection = [];
        edge_to_break = [];
    case 'FCA_priority'
        obj = FCA_priority(scenario,iter);
        [veh_at_intersection, groups, directed_adjacency] = obj.priority();
        right_of_way = false;
        edge_to_break = [];   
end

    % visualize the coupling between vehicles
%     plot_coupling_lines(directed_adjacency, iter)

    % construct the priority list
    computation_levels = length(groups);
    members_list = horzcat(groups.members);
    nVeh = length(members_list); 
    priority_list = zeros(1,nVeh);
    prio = 1;
    for iVeh = members_list 
        priority_list(iVeh) = prio;
        prio = prio + 1;
    end
    
    % update properties of scenario
    scenario.directed_coupling = directed_adjacency;
    scenario.priority_list = priority_list;
    scenario.last_veh_at_intersection = veh_at_intersection;

    y_pred = cell(scenario.nVeh,1);
    u = zeros(scenario.nVeh,1);
    
    info = struct;
    info.vehicle_fullres_path = cell(scenario.nVeh,1);
    info.trim_indices = (-1)*ones(scenario.nVeh,1);
    info.subcontroller_runtime = zeros(scenario.nVeh,1);
    info.shapes = cell(scenario.nVeh,scenario.Hp);
    info.next_node = node(-1, zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), zeros(scenario.nVeh,1), -1, -1);
    info.n_expanded = 0;
    info.priority_list = priority_list;
    info.veh_at_intersection = veh_at_intersection;
    info.computation_levels = computation_levels;
    info.edge_to_break = edge_to_break;
    
    % graph-search to select the optimal motion primitive
    sub_controller = @(scenario, iter)...
        graph_search(scenario, iter); 

    for grp_idx = 1:length(groups)
        group = groups(grp_idx);
        for grp_member_idx = 1:length(group.members) 
            subcontroller_timer = tic;
            vehicle_idx = group.members(grp_member_idx);
%             if vehicle_idx==6
%                 disp('debug')
%             end
            % Filter out vehicles that are not adjacent
            veh_adjacent = find(scenario.adjacency(vehicle_idx,:,end));
            predecessors = intersect(group.predecessors,veh_adjacent);
%             predecessors = group.predecessors;

            % Filter out vehicles with lower or same priority.
            priority_filter = false(1,scenario.nVeh);
            priority_filter(predecessors) = true; % keep all with higher priority
            priority_filter(vehicle_idx) = true; % keep self
            scenario_filtered = filter_scenario(scenario, priority_filter);
            iter_filtered = filter_iter(iter, priority_filter);

            self_index = sum(priority_filter(1:vehicle_idx));        
            v2o_filter = true(1,scenario_filtered.nVeh);
            v2o_filter(self_index) = false;

            % add predicted trajecotries of vehicles with higher priority as dynamic obstacle
            [scenario_v, iter_v] = vehicles_as_dynamic_obstacles(scenario_filtered, iter_filtered, v2o_filter, info.shapes(predecessors,:));
            
            % add adjacent vehicles with lower priorities as static obstacles
            if right_of_way
                adjacent_vehicle_lower_priority = setdiff(veh_adjacent,predecessors);
                
                % only two strategies are supported if parallel computation is not used
                assert(strcmp(scenario_v.strategy_consider_veh_with_lower_prio,'1')==true || strcmp(scenario_v.strategy_consider_veh_with_lower_prio,'4')==true)
                scenario_v = consideration_of_followers_by_leader(scenario_v, iter, adjacent_vehicle_lower_priority);
%                 scenario_v = vehicles_as_static_obstacles(scenario_v,iter,adjacent_vehicle_lower_priority);
            end
%             if scenario_v.k >= 12 && vehicle_idx==1
%                 disp('') % debug
%                 plot_obstacles(scenario_v)
%             end
            % execute sub controller for 1-veh scenario
            [u_v,y_pred_v,info_v] = sub_controller(scenario_v, iter_v);
            
            % prepare output data
            info.tree{vehicle_idx} = info_v.tree;
            info.tree_path(vehicle_idx,:) = info_v.tree_path;
            info.subcontroller_runtime(vehicle_idx) = toc(subcontroller_timer);
            info.n_expanded = info.n_expanded + info_v.tree.size();
            info.next_node = set_node(info.next_node,vehicle_idx,info_v);
            info.shapes(vehicle_idx,:) = info_v.shapes(:);
            info.vehicle_fullres_path(vehicle_idx) = path_between(info_v.tree_path(1),info_v.tree_path(2),info_v.tree,scenario);
            info.trim_indices(vehicle_idx) = info_v.trim_indices;
            info.predicted_trims(vehicle_idx,:) = info_v.predicted_trims; % store the planned trims in the future Hp time steps
            info.trees{vehicle_idx} = info_v.tree; % store tree information
            info.y_predicted{vehicle_idx,1} = y_pred_v{:}; % store the information of the predicted output
            y_pred{vehicle_idx,1} = y_pred_v{:};
            u(vehicle_idx) = u_v(1);
        end

    end

    % calculate the total run time: only one vehicle in each computation level will be counted, this is the one with the maximum run time 
    subcontroller_run_time_total = 0;
    for level_i = 1:computation_levels
        vehs_in_level_i = groups(level_i).members;
        subcontroller_run_time_total = subcontroller_run_time_total + max(info.subcontroller_runtime(vehs_in_level_i));
    end

    info.subcontroller_run_time_total = subcontroller_run_time_total;
   
end
