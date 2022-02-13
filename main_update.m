function result = main_update(varargin)
% MAIN  main function for graph-based receeding horizon control with
% right-of-way priority assignment. Vehicles at intersection keep
% higher priority and do not change their relative priority until they leave intersection

if verLessThan('matlab','9.10')
    warning("Code is developed in MATLAB 2021a, prepare for backward incompatibilities.")
end

%% Determine options
% if matlab simulation should be started with certain parameters
% first argument has to be 'sim'
is_sim_lab = (nargin == 0 || (nargin > 0 && strcmp(varargin{1},'sim')));

if is_sim_lab
    switch nargin
        case 4
            options = selection(varargin{2},varargin{3},varargin{4});
        case 3
            options = selection(varargin{2},varargin{3},1);
        case 2
            options = selection(varargin{2},2,1);
        otherwise
            options = selection();
    end

    vehicle_ids = 1:20; % ok
%     vehicle_ids = 22:33; % vheicles running in a circle(test for cyclic directed graph)
else
    disp('cpmlab')
    options = struct;
    vehicle_ids = [varargin{:}];
    options.amount = numel(vehicle_ids);
    options.isPB = true;
end
  

scenario = commonroad(options.amount,vehicle_ids,options.isPB);

% scenario = lanelet_scenario4(options.isPB);

% scenario = circle_scenario(options.amount,options.isPB);

if is_sim_lab
    exp = SimLab(scenario, options);
else
    exp = CPMLab(scenario, vehicle_ids);
end


%% Setup
% Initialize
got_stop = false;
k = 1;

% init result struct
result = get_result_struct(scenario);

exp.setup();
last_veh_at_intersection = [];
fallback=1;
%% Main control loop
while (~got_stop)
    
    result.step_timer = tic;
    % Measurement
    % -------------------------------------------------------------------------
    [ x0, trim_indices ] = exp.measure();% trim_indices： which trim  

%     disp('x0 is:')
%     disp(x0)
    try
        % Control 
        % ----------------------------------------------------------------------
        % Sample reference trajectory
        iter = rhc_init(scenario,x0,trim_indices);
        
        if ~isempty(scenario.lanelets)
            lanelet_boundary = lanelets_boundary(scenario, iter);% update the boundary information of each vehicle
            for iveh = 1:options.amount
                scenario.vehicles(1,iveh).lanelet_boundary = lanelet_boundary{1,iveh};
            end
            [scenario,scenario.adjacency(:,:,k),scenario.semi_adjacency(:,:,k)] = coupling_adjacency(scenario,iter);
        end
        
%         % calculate the distance 
        distance = zeros(options.amount,options.amount);
        adjacency = scenario.adjacency(:,:,k);

        for vehi = 1:options.amount-1
            adjacent_vehicle = find(adjacency(vehi,:));
            adjacent_vehicle = adjacent_vehicle(adjacent_vehicle > vehi);
            for vehn = adjacent_vehicle
                distance(vehi,vehn) = check_distance(iter,vehi,vehn);
            end
        end
        
        result.distance(:,:,k) = distance;
        
        
%         disp('adjacency_matrix is:')
%         disp(scenario.adjacency)
        scenario_tmp = get_next_dynamic_obstacles_scenario(scenario, k);
        result.iter_runtime(k) = toc(result.step_timer);
        result.scenario = scenario;
        
        
        controller_timer = tic;
            [u, y_pred, info, priority_list, veh_at_intersection,computation_levels,edge_to_break] = scenario.controller(scenario_tmp, iter, last_veh_at_intersection);
        
%         disp(['Hi priority: ',num2str(priority_list)])

        last_veh_at_intersection = veh_at_intersection;
        
        result.controller_runtime(k) = toc(controller_timer);
        result.iteration_structs{k} = iter;
        % save controller outputs in result struct
        result.trajectory_predictions(:,k) = y_pred;
        result.controller_outputs{k} = u;
        result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
        % store vehicles path in higher resolution
        result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
        result.n_expanded(k) = info.n_expanded;
        result.step_time(k) = toc(result.step_timer);
        result.priority(:,k) = priority_list;
        result.computation_levels(k) = computation_levels;
        result.edges_to_break{k} = edge_to_break;
        
        % Apply control action f/e veh
        % -------------------------------------------------------------------------
        apply_timer = tic;
        exp.apply(u, y_pred, info, result, k);
        result.apply_runtime(k) = toc(apply_timer);
        result.total_runtime(k) = toc(result.step_timer);
        
        
    % catch case where graph search could not find a new node
    catch ME
        switch ME.identifier
        case 'MATLAB:graph_search:tree_exhausted'
%             warning([ME.message, ', ending search...']);

%             disp('ME, fallback to last priority...............................')  
            warning([ME.message, ', ME, fallback to last priority.............']);
            controller_timer = tic;
            [u, y_pred, info] = pb_controller_fallback(scenario, u, y_pred, info);

            result.controller_runtime(k) = toc(controller_timer);
            result.iteration_structs{k} = iter;
            % save controller outputs in result struct
            result.trajectory_predictions(:,k) = y_pred;
            result.controller_outputs{k} = u;
            result.subcontroller_runtime(:,k) = info.subcontroller_runtime;
            % store vehicles path in higher resolution
            result.vehicle_path_fullres(:,k) = info.vehicle_fullres_path(:);
            result.n_expanded(k) = info.n_expanded;
            result.step_time(k) = toc(result.step_timer);
            result.priority(:,k) = priority_list;
            result.computation_levels(k) = computation_levels;

            % Apply control action f/e veh
            % -------------------------------------------------------------------------
            apply_timer = tic;
            exp.apply(u, y_pred, info, result, k);
            result.apply_runtime(k) = toc(apply_timer);
            result.total_runtime(k) = toc(result.step_timer);
            disp('plotting finished ........................................');  
            fallback = fallback + 1;
          
%             got_stop = true;
        otherwise
            rethrow(ME)
        end
    end
    
    % Check for stop signal
    % -------------------------------------------------------------------------
    got_stop = exp.is_stop() || got_stop;
    
    % increment interation counter
    k = k+1;
end
%% save results
disp(['fallback times: ',num2str(fallback)])
save(fullfile(result.output_path,'data.mat'),'result');
% exportVideo( result );
exp.end_run()
% a;
end
