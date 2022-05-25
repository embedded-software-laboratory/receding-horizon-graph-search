classdef MotionPrimitiveAutomaton
% MOTIONPRIMITVEAUTOMATON   MotionPrimitiveAutomaton 
   
    properties
        maneuvers % cell(n_trims, n_trims)
        trims % A struct array of the specified trim_inputs
        transition_matrix_single % Matrix (nTrims x nTrims x horizon_length)
        trim_tuple               % Matrix with trim indices ((nTrims1*nTrims2*...) x nVehicles)
        transition_matrix        % binary Matrix (if maneuverTuple exist according to trims) (nTrimTuples x nTrimTuples x horizon_length)
        distance_to_equilibrium  % Distance in graph from current state to equilibrium state (nTrims x 1)
        recursive_feasibility
        local_reachable_sets           % Reachable sets of each trim (possibly non-convex); 
                                            % It's local because the position and yaw angle of vehicles are not considered
        local_reachable_sets_conv;     % Convexified reachable sets of each trim
    end
    
    methods
        function obj = MotionPrimitiveAutomaton(model, trim_set, offset, dt, nveh, N, nTicks, recursive_feasibility, options)
            % Constructor
            % trim_inputs is a matrix of size (nTrims x nu)
            % trim_adjacency is a matrix of size (nTrims x nTrims), 
            %   read as: rows are start trims and columns are end trims
            % N is the horizon length

            % path of the MPA
            [file_path,~,~] = fileparts(mfilename('fullpath'));
            folder_target = [file_path,filesep,'library'];
            if ~isfolder(folder_target)
                % create target folder if not exist
                mkdir(folder_target)
            end
            if options.isParl
                mpa_instance_name = ['MPA_','trims',num2str(trim_set),'_Hp',num2str(N),'_parl','.mat'];
            else
                mpa_instance_name = ['MPA_','trims',num2str(trim_set),'_Hp',num2str(N),'.mat'];
            end
            mpa_full_path = [folder_target,filesep,mpa_instance_name];

            % if the needed MPA is alread exist in the library, simply load
            % it, otherwise it will be calculated and saved to the library.
            if isfile(mpa_full_path)
                load(mpa_full_path,"mpa");
                obj = mpa;
                return
            end

            obj.recursive_feasibility = recursive_feasibility;
                        
            [trim_inputs, trim_adjacency] = choose_trims(trim_set);
            n_trims = length(trim_inputs);
            
            obj.transition_matrix_single = zeros([size(trim_adjacency),N]);
            obj.transition_matrix_single(:,:,:) = repmat(trim_adjacency,1,1,N);
            
            % trim struct array
            % BicycleModel
            if model.nu == 2
                obj.trims = struct('steering',0,'speed',0); 
            % BicycleModelConstSpeed
            elseif model.nu == 1
                obj.trims = struct('steering',0);
            end 
            
            for i = 1:n_trims
                obj.trims(i) = generate_trim(model, trim_inputs(i,:));
            end
            
            
            % maneuver cell/struct matrix
            for i = 1:n_trims
                for j = 1:n_trims
                    if obj.transition_matrix_single(i,j,1)
                        obj.maneuvers{i,j} = generate_maneuver(model, obj.trims(i), obj.trims(j), offset, dt, nTicks);
                    end
                end
            end

            % compute distance to equilibrium state
            eq_states = find(trim_inputs(:,2)==0);            
            adj_trims = graph(obj.transition_matrix_single(:,:,1));
            obj.distance_to_equilibrium = distances(adj_trims,eq_states);

            % compute trim tuple (vertices)
            trim_index_list = cell(nveh,1);
            [trim_index_list{:}] = deal(1:n_trims);
            obj.trim_tuple = cartprod(trim_index_list{:});
            
            if recursive_feasibility
                obj.transition_matrix_single = compute_time_varying_transition_matrix(obj);
            end

            % compute maneuver matrix for trimProduct
            obj.transition_matrix = compute_product_maneuver_matrix(obj,nveh,N);
            
            % variables to store reachable sets in different time steps
            obj.local_reachable_sets = cell(n_trims,N);
            obj.local_reachable_sets_conv = cell(n_trims,N);
                
            % For parallel computation, reachability analysis are used
            if options.isParl
                [obj.local_reachable_sets, obj.local_reachable_sets_conv] = reachability_analysis_offline(obj,N);
            end
            
            save_mpa(obj,mpa_full_path); % save mpa to library
            
        end
    
        function max_speed = get_max_speed(obj, cur_trim_id)
            % returns maximum speed, averaged over the timestep (nSamples x 1)
            % is not general, but works for current MPAs
            N = size(obj.transition_matrix,3);
            max_speed = zeros(N,1);
            max_speed_last = obj.trims(cur_trim_id).speed;
            for k = 1:N
                successor_trim_ids = find(obj.transition_matrix_single(cur_trim_id, :, k));
                [max_speed_next, cur_trim_id] = max([obj.trims(successor_trim_ids).speed]);
                max_speed(k) = (max_speed_next + max_speed_last)/2; % assumes linear change
                max_speed_last = max_speed_next;
            end
        end

        
        function transition_matrix_single = compute_time_varying_transition_matrix(obj)
            N = size(obj.transition_matrix_single,3);
            transition_matrix_single = obj.transition_matrix_single;
            for k = 1:N
                % Columns are end trims. Forbid trims whose distance 
                % to an equilbrium state is too high
                k_to_go = N-k;
                transition_matrix_single(:,obj.distance_to_equilibrium>k_to_go,k) = 0;
            end
        end


        function maneuver_matrix = compute_product_maneuver_matrix(obj,nveh,N)
            nTrimTuples = size(obj.trim_tuple,1);
            maneuver_matrix = zeros(nTrimTuples,nTrimTuples,N);
            % Assumes Hp=Hu
            for k = 1:N
                transition_matrix_slice = obj.transition_matrix_single(:,:,k);
                % compute tensor product iteratively
                for i = 2 : nveh
                    transition_matrix_slice = kron(transition_matrix_slice,obj.transition_matrix_single(:,:,k));
                end
                maneuver_matrix(:,:,k) = transition_matrix_slice;
            end
        end

        function [reachable_sets_local, reachable_sets_conv_local] = reachability_analysis_offline(obj, Hp)
            % Calculate local reachable sets starting from a certain trim,
            % which can be used for online reachability analysis
            % 
            % INPUT:
            %   obj: motion primitive automaton calss
            %   
            %   Hp: prediction horizon
            % 
            % OUTPUT:
            %   reachable_sets_local: cell [n_trims x Hp]. The union of local reachable
            %   sets  
            %     
            %   reachable_sets_conv_local: cell [n_trims x Hp]. The convexified union
            %   of local reachable sets 
            
            threshold_Hp = 5;
            if Hp > threshold_Hp
                warning(['Computing the reachable sets now...' newline ...
                    'Since the prediction horizon is ' num2str(Hp) ' (more than ' num2str(threshold_Hp) '), it may take several minutes.' newline ...
                    'Note this only needs to be done once since later they will be saved for offline use.'])
            end
        
            n_trims = numel(obj.trims);
            reachable_sets_local = cell(n_trims,Hp);
            reachable_sets_conv_local = cell(n_trims,Hp);
        
            % transform maneuver area to polyshape which is required when using
            % MATLAB function `union`
            for i=1:n_trims
                child_trims = find(obj.transition_matrix_single(i,:,1));
                for idx=1:length(child_trims)
                    j = child_trims(idx);
                    obj.maneuvers{i,j}.areaPoly = polyshape(obj.maneuvers{i,j}.area(1,:),obj.maneuvers{i,j}.area(2,:),'Simplify',false);
                end
            end
            
%             % todo?: use a different horizon for reachability analysis
%             if Hp>5
%                 Hp = 5;
%                 warning(['The pridiction horizon is too large for reachability analysis and therefore a prediction horizon of ', num2str(Hp),' will be used.'])
%             end
        
            trimsInfo = struct;
            for i=1:n_trims
                for t=1:Hp
                    if t==1 % root trim
                        trimsInfo(i,t).parentTrims = i;
                    else % The child trims become the parent trims of the next time step
                        trimsInfo(i,t).parentTrims = trimsInfo(i,t-1).childTrims;
                    end
                    
                    % variable to store all child trims of the parent trims
                    trimsInfo(i,t).childTrims = [];
                    trimsInfo(i,t).childNum = [];
                    % find child trims of the parent trims
                    for i_Trim=1:length(trimsInfo(i,t).parentTrims)
                        find_child = find(obj.transition_matrix_single(trimsInfo(i,t).parentTrims(i_Trim),:,t));
                        trimsInfo(i,t).childTrims = [trimsInfo(i,t).childTrims find_child];
                        trimsInfo(i,t).childNum = [trimsInfo(i,t).childNum length(find_child)];
                    end
                    
                    % store the union of the reachable sets of the parent trims in the prediction horizon
                    trimsInfo(i,t).reachable_sets = polyshape;
                    % store the union of the reachable sets of the parent trim
                    reachable_sets_union = cell(1,length(trimsInfo(i,t).parentTrims));
                    
                    % loop through all parent trims
                    for j=1:length(trimsInfo(i,t).parentTrims)
                        reachable_sets_union{j} = polyshape;
                        % loop through all child trims
                        for k=1:trimsInfo(i,t).childNum(j)
                            trim_start = trimsInfo(i,t).parentTrims(j);
                            child_ordinal = sum(trimsInfo(i,t).childNum(1:j-1)) + k;
                            trim_end = trimsInfo(i,t).childTrims(child_ordinal);
                            if t==1
                                x0 = 0;
                                y0 = 0;
                                yaw0 = 0;
                            else
                                x0 = trimsInfo(i,t-1).maneuvers{j}.dx;
                                y0 = trimsInfo(i,t-1).maneuvers{j}.dy;
                                yaw0 = trimsInfo(i,t-1).maneuvers{j}.dyaw;
                            end
            
                            % tranlates the local coordinates to global coordinates
                            [trimsInfo(i,t).maneuvers{child_ordinal}.xs, trimsInfo(i,t).maneuvers{child_ordinal}.ys] = ...
                                translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.xs,obj.maneuvers{trim_start,trim_end}.ys);
                            trimsInfo(i,t).maneuvers{child_ordinal}.yaws = yaw0 + obj.maneuvers{trim_start,trim_end}.yaws;
                            trimsInfo(i,t).maneuvers{child_ordinal}.dx = trimsInfo(i,t).maneuvers{child_ordinal}.xs(end);
                            trimsInfo(i,t).maneuvers{child_ordinal}.dy = trimsInfo(i,t).maneuvers{child_ordinal}.ys(end);
                            trimsInfo(i,t).maneuvers{child_ordinal}.dyaw = trimsInfo(i,t).maneuvers{child_ordinal}.yaws(end);
            
                            % occupied area of the translated maneuvers
                            [area_x, area_y] = ...
                                translate_global(yaw0,x0,y0,obj.maneuvers{trim_start,trim_end}.area(1,:),obj.maneuvers{trim_start,trim_end}.area(2,:));
                            trimsInfo(i,t).maneuvers{child_ordinal}.area = [area_x;area_y];
                            trimsInfo(i,t).maneuvers{child_ordinal}.areaPoly = polyshape(area_x,area_y,'Simplify',false);
            
                            % union of the reachable sets of one parent trim
                            reachable_sets_union{j} = union(reachable_sets_union{j},trimsInfo(i,t).maneuvers{child_ordinal}.areaPoly);
                        end
                        % union of the reachable sets of all parent trims
                        trimsInfo(i,t).reachable_sets = union(trimsInfo(i,t).reachable_sets,reachable_sets_union{j});
                        trimsInfo(i,t).reachable_sets_conv = convhull(trimsInfo(i,t).reachable_sets);
                    end
                    reachable_sets_local{i,t} = trimsInfo(i,t).reachable_sets;
                    reachable_sets_conv_local{i,t} = trimsInfo(i,t).reachable_sets_conv;
                end
            end
        
        end

        function save_mpa(obj,mpa_full_path)
            % Save MPA to library
            mpa = obj;
            save(mpa_full_path,'mpa');
        end

        function emergency_braking_distance = get_emergency_braking_distance(obj, cur_trim_id, time_step)
            % returns the emergency braking distance starting from the current trim
            emergency_braking_distance = 0;
            speed_cur = obj.trims(cur_trim_id).speed;

            % compute the shortest path from the current trim to the equilibrium trim
            equilibrium_trim = find([obj.trims.speed]==0);
            assert(length(equilibrium_trim)==1) % if there are multiple equilibrium states, this function should be then adapted
            graph_trims = graph(obj.transition_matrix_single(:,:,1));
            shortest_path_to_equilibrium = shortestpath(graph_trims,cur_trim_id,equilibrium_trim); % shortest path between current trim and equilibrium trim

            for iTrim=shortest_path_to_equilibrium(2:end)
                speed_next = obj.trims(iTrim).speed;
                speed_mean = (speed_cur+speed_next)/2; % assume linear change
                emergency_braking_distance = emergency_braking_distance + speed_mean*time_step;
                speed_cur = speed_next; % update the current speed for the next iteration
            end
        end
        
        function shortest_time_to_arrive = get_the_shortest_time_to_arrive(obj, cur_trim_id, distance_destination, time_step)
            % Returns the shortest time to arive a given distance starting from the current trim
            % Noted that this is only a lower bound time because the
            % steering angle is not considered, namely we assume the
            % vehicle drives straight to arrive the goal destination.
            shortest_time_to_arrive = 0;
            distance_remained = distance_destination;
            distance_acceleration = 0; % acceleration distance
            % compute the shortest path from the current trim to the
            % trim(s) with maximum speed
            max_speed = max([obj.trims.speed]);
            max_speed_trims = find([obj.trims.speed]==max_speed); % find all the trims with the maximum speed

            graph_trims = graph(obj.transition_matrix_single(:,:,1));
            shortest_distances_to_max_speed = distances(graph_trims,cur_trim_id,max_speed_trims); % shortest path between two single nodes
            % find the one which has the minimal distance to the trims with the maximum speed
            [min_distance,idx] = min(shortest_distances_to_max_speed); 
            if min_distance==0 % if the current trim has already the maximum speed, no acceleration is needed
                shortest_time_to_arrive = distance_remained/max_speed;
            else % acceleration to maximum speed
                max_speed_trim = max_speed_trims(idx);
                shortest_path_to_max_speed = shortestpath(graph_trims,cur_trim_id,max_speed_trim); % shortest path between two single nodes
                for i=1:length(shortest_path_to_max_speed)
                    trim_current = shortest_path_to_max_speed(i);
                    
                    speed_cur = obj.trims(trim_current).speed;
                    if i+1<=length(shortest_path_to_max_speed)
                        trim_next = shortest_path_to_max_speed(i+1);
                        speed_next = obj.trims(trim_next).speed;
                    else
                        speed_next = max_speed;
                    end
                    mean_speed = (speed_cur+speed_next)/2;
                    distance_acceleration = distance_acceleration + mean_speed*time_step;
                    if distance_acceleration > distance_destination % if the vehicle arrives the detination when accelerating
                        shortest_time_to_arrive = shortest_time_to_arrive + distance_remained/mean_speed; % time accumulates
                        break
                    else
                        shortest_time_to_arrive = shortest_time_to_arrive + time_step; % time accumulates
                        distance_remained = distance_destination - distance_acceleration;
                    end
                end

                % if the detination is still not arrived after acceleration, calculate the remaining time using the maximum speed
                if distance_remained>0
                    shortest_time_to_arrive = shortest_time_to_arrive + distance_remained/max_speed;
                end

            end
        end
    end
end

