function scenario = get_coupling_infos(scenario, iter)
% GET_COUPLING_INFOS This function estimate the coupling information of
% vehicles. Two vehicles are coupled if their reachable sets in the defined 
% prediction horizon (Hp) overlap. To reduce the checking effort, only the
% reachable sets in the last time step will be checked, which is almost
% sufficient. Heuristic method will be used to estimate the coupling degree
% between vehicles. More concrete,, the so-called "the shortest time to
% achieve a collision (STAC)" will be calculated, whilea a lower STAC means
% a higher coupling degree. 

    lanelets = scenario.lanelets;
    lanelet_relationships = scenario.lanelet_relationships;
%     adjacency = zeros(nVeh,nVeh);
%     semi_adjacency = zeros(nVeh,nVeh);
    
    nVeh = scenario.nVeh;
    STAC = zeros(nVeh,nVeh); % the shortest time to achieve a collision
    coupling_weights = zeros(nVeh,nVeh); % coupling weights of all coupling vehicle pair; higher value indicates stronger coupling
    
    % initialize a struct array to store information about coupling information
    coupling_infos = struct('idLeader',[],'idFollower',[],'type',[],'speedLeader',[],'speedFollower',[],'positionLeader',[],'positionFollower',[],'STAC',[],'weight',[]); 
    count = 1;

    state_indices = indices();
    
    for veh_i = 1:(nVeh-1)
        % get the selected vehicle's position, speed, current trim and predicted lanelets 
        speed_i = iter.x0(veh_i, state_indices.speed);
        position_i = [iter.x0(veh_i,state_indices.x), iter.x0(veh_i,state_indices.y)];
        trim_i = iter.trim_indices(veh_i);
        reachable_sets_last_step_i = iter.reachable_sets{veh_i,end};
        predicted_lanelets_i = iter.predicted_lanelets{veh_i};

        for veh_j = (veh_i+1):nVeh
            speed_j = iter.x0(veh_j, state_indices.speed);
            position_j = [iter.x0(veh_j,state_indices.x), iter.x0(veh_j,state_indices.y)];
            trim_j = iter.trim_indices(veh_j);
            reachable_sets_last_step_j = iter.reachable_sets{veh_j,end};
            predicted_lanelets_j = iter.predicted_lanelets{veh_j};

            if overlaps(reachable_sets_last_step_i, reachable_sets_last_step_j)
                stop_flag = false;

                % the selected two vehicles are considered as coupled if their reachable sets overlap
                for k_i = 1:length(predicted_lanelets_i)  
                    predicted_lanelet_i = predicted_lanelets_i(k_i);
                    
%                     % debug
%                     predicted_lanelet_i = 87;
%                     position_i = [0.3,1.4];
%                     position_j = [0.25,1.1];
%                     trim_i = 7;
%                     trim_j = 7;

                    % center line of the lanelet
                    lanelet_x_i = lanelets{predicted_lanelet_i}(:,LaneletInfo.cx);
                    lanelet_y_i = lanelets{predicted_lanelet_i}(:,LaneletInfo.cy);
                    for k_j = 1:length(predicted_lanelets_j)
                        predicted_lanelet_j = predicted_lanelets_j(k_j);

%                         % debug
%                         predicted_lanelet_j = 85;

                        % center line of the lanelet
                        lanelet_x_j = lanelets{predicted_lanelet_j}(:,LaneletInfo.cx);
                        lanelet_y_j = lanelets{predicted_lanelet_j}(:,LaneletInfo.cy);
                        
                        % get the relationship of their predicted lanelets and the possible collision type 
                        if predicted_lanelet_i == predicted_lanelet_j
                            is_same_lanelet = true; % the same lanelet
                            % If the same lanelet -> rear-end collision
                            is_rear_end_collision = true;
                            is_side_impact_collision = false;
                            is_forking_lanelets = false;
                        else
                            is_same_lanelet = false;
                            % find if there exists lanelet pair that has a certain relationship in the struct array `lanelet_relationships`
                            % NOTE that only adjacent lanelet pairs with certain relationships will be stored in `lanelet_relationships`
                            idx_lanelet_pair = find_idx_lanelet_pair(predicted_lanelet_i,predicted_lanelet_j,lanelet_relationships);
                            if ~isempty(idx_lanelet_pair)
                                lanelet_relationship = lanelet_relationships(idx_lanelet_pair);

                                % If forking lanelets (relationship type 4) or successive (relationship type 1) lanelets or 
                                % adjacent left/right lanelets (relationship type 2) -> rear-end collision
                                is_forking_lanelets = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_4);
                                is_rear_end_collision = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_1) ||...
                                    strcmp(lanelet_relationship.type, LaneletRelationshipType.type_2) || is_forking_lanelets;
                                
                                % if merging (relationship type 3) lanelets or intersecting lanelets (relationship type 5) -> side-impact collision
                                is_side_impact_collision = strcmp(lanelet_relationship.type, LaneletRelationshipType.type_3) ||...
                                    strcmp(lanelet_relationship.type, LaneletRelationshipType.type_5);

                            else
                                continue
                            end
                        end

                        if is_rear_end_collision
                            % If the same lanelet or successive (relationship type 1) lanelets or adjacent left/right lanelets (relationship type 2) -> rear-end collision
                            coupling_infos(count).type = CollisionType.type_1; % rear-end collision

                            if is_forking_lanelets
                                % In forking lanelets, rear-end collision is possible around the forking point
                                % First, let the leader take an emergency
                                % braking maneuver and see if the stop
                                % point is invariant safe for vehicle
                                % driving in another forking lanelet. If
                                % yes, no collision is possible. If no, the
                                % STAC is the time that the follower needs
                                % to arrive the point to collide with the
                                % leader at the stopping point.

                                forking_point = lanelet_relationship.point;
                                
                                distance_to_starting_point_i = norm(position_i-forking_point,2);
                                distance_to_starting_point_j = norm(position_j-forking_point,2);
    
                                % vehicle closer to the forking point is the leader
                                if distance_to_starting_point_i >= distance_to_starting_point_j
                                    is_leader = true;
                                    trim_leader = trim_i;
                                    position_leader = position_i;
                                    trim_follower = trim_j;
                                    position_follower = position_j;
                                    lanelet_leader = [lanelet_x_i,lanelet_y_i];
                                    lanelet_follower = [lanelet_x_j,lanelet_y_j];
                                else
                                    is_leader = false;
                                    trim_leader = trim_j;
                                    position_leader = position_j;
                                    trim_follower = trim_i;
                                    position_follower = position_i;
                                    lanelet_leader = [lanelet_x_j,lanelet_y_j];
                                    lanelet_follower = [lanelet_x_i,lanelet_y_i];
                                end

                                % let the leader take an emergency breaking maneuver
                                emergency_braking_distance = get_emergency_braking_distance(scenario.mpa, trim_leader, scenario.dt);

                                % current distance between two vehicles
                                cur_distance = norm(position_i-position_j,2);
                                % the total distance to collision is the sum of the current distance between two vehicles and the emergency braking distance of the leader
                                distance_to_collision = cur_distance + emergency_braking_distance;
                                % let the follower take a full acceleration
                                % to catch the leader, then calculte the STAC if two vehicles are driving at the same lanelet 
                                [STAC_tmp, distance_traveled_leader, distance_traveled_follower] = scenario.mpa.get_the_shortest_time_to_catch(trim_leader, trim_follower, distance_to_collision, scenario.dt);

                                stop_point_leader = get_stop_point_after_travel_certain_distance(position_leader,lanelet_leader,distance_traveled_leader);
                                stop_point_follower = get_stop_point_after_travel_certain_distance(position_follower,lanelet_follower,distance_traveled_follower);

                                distance_stop_points = norm(stop_point_leader-stop_point_follower);
                                if distance_stop_points > Vehicle().Length
                                    % the selected two vehicles will never collide with each other since their stopping points are far away
                                    continue
                                end
                            else
                                % Otherwise vehicles are driving at the same lanelets or successive lanelets 
                                
                                % Get the endpoint of the lanelet
                                if is_same_lanelet
                                    endpoint = [lanelet_x_i(end), lanelet_y_i(end)];
                                else
                                    endpoint = lanelet_relationship.point;
                                end
    
                                distance_to_endpoint_i = norm(position_i-endpoint,2);
                                distance_to_endpoint_j = norm(position_j-endpoint,2);
    
                                if distance_to_endpoint_i <= distance_to_endpoint_j
                                    is_leader = true;
                                    % vehicle_i is the leader and thus takes an emergency braking maneuver
                                    emergency_braking_distance = scenario.mpa.get_emergency_braking_distance(trim_i, scenario.dt);
                                    trim_follower = trim_j;
                                else
                                    is_leader = false;
                                    % else vehicle_j is the leader and thus takes an emergency braking maneuver
                                    emergency_braking_distance = scenario.mpa.get_emergency_braking_distance(trim_j, scenario.dt);
                                    trim_follower = trim_i;
                                end
    
                                % current distance between two vehicles
                                cur_distance = norm(position_i-position_j,2);
                                % the total distance to collision is the sum of the current distance between two vehicles and the emergency braking distance of the leader
                                distance_to_collision = cur_distance + emergency_braking_distance;
                                % calculate the shortest time to achieve a collision 
                                STAC_tmp = get_the_shortest_time_to_arrive(scenario.mpa, trim_follower, distance_to_collision, scenario.dt);
                            end


                        elseif is_side_impact_collision
                            coupling_infos(count).type = CollisionType.type_2; % side-impact collision

                            % For side-impact collision, we calculate the adapted STAC, which is the sum of the actual STAC and a waiting time. 
                            % The calculation procedure is the following: Firstly, the actual STAC is the shortest time needed to achieve a 
                            % collision at the point of intersection. This is calculated by letting both vehicles take a full acceleration to 
                            % arrive at the point of intersection. The first arrived vehicle has to stop at the point of intersection and wait 
                            % for the second arrived vehicle, where the waiting time is the second part of the adapted STAC.
                            collision_point = lanelet_relationship.point;
                            
                            % from a curve to calculate the arc diatance between vehicle's current position and the collision point, 
                            % which starts from the starting point of the lanelet and ends at the collision point 
                            if strcmp(lanelet_relationship.type,LaneletRelationshipType.type_3)
                                % collision point of the merging lanelets is both lanelets' endpoint, thus the target curve is the whole lanelet
                                curve_x_i = lanelet_x_i;
                                curve_y_i = lanelet_y_i;
                                curve_x_j = lanelet_x_j;
                                curve_y_j = lanelet_y_j;
                            elseif strcmp(lanelet_relationship.type,LaneletRelationshipType.type_5)
                                % Collision point of the intersecting lanelets is the crosspoint, which could be in the middle of the lanelets
                                % First, find the two closest points to the crosspoint on the lanelet
                                squared_distances_to_crosspoint_i = sum(([lanelet_x_i,lanelet_y_i]-collision_point).^2,2);
                                [~,idx_closest_two_point_i] = mink(squared_distances_to_crosspoint_i,2,1);
                                squared_distances_to_crosspoint_j = sum(([lanelet_x_j,lanelet_y_j]-collision_point).^2,2);
                                [~,idx_closest_two_point_j] = mink(squared_distances_to_crosspoint_j,2,1);
                                % the endpoint of the curve is the collision point and the adjacent left point is the one among the closest two points with a smaller index  
                                curve_x_i = [lanelet_x_i(1:min(idx_closest_two_point_i));collision_point(1)];
                                curve_y_i = [lanelet_y_i(1:min(idx_closest_two_point_i));collision_point(2)];
                                curve_x_j = [lanelet_x_j(1:min(idx_closest_two_point_j));collision_point(1)];
                                curve_y_j = [lanelet_y_j(1:min(idx_closest_two_point_j));collision_point(2)];
                            end

                            % calculate the arc length from the vehicle's current position to the collision point
                            [arc_distance_to_collision_i, arc_length_i, ~, ~, ~] = get_arc_distance_to_endpoint(position_i(1), position_i(2), curve_x_i, curve_y_i);
                            [arc_distance_to_collision_j, arc_length_j, ~, ~, ~] = get_arc_distance_to_endpoint(position_j(1), position_j(2), curve_x_j, curve_y_j);
                            
                            if arc_length_i == 0 || arc_length_j == 0
                                % one vehile has already passed the point of intersection; thus, collision is no longer possible
                                continue
                            end

                            time_to_collisionPoint_i = get_the_shortest_time_to_arrive(scenario.mpa,trim_i,arc_distance_to_collision_i,scenario.dt);
                            time_to_collisionPoint_j = get_the_shortest_time_to_arrive(scenario.mpa,trim_j,arc_distance_to_collision_j,scenario.dt);

                            if arc_distance_to_collision_i <= arc_distance_to_collision_j
                                % vehicle closer to the collision point has a higher priority (or also called as "leader")
                                is_leader = true; 
                            else
                                is_leader = false; 
                            end

                            % the first arrived vehicle will wait for the second arrived vehicle to "achieve" a collision.
                            % Here we ignore the fact that the first arrived vehicle should also decelerate to stop at the
                            % point of intersection, which could be added to todo list to improve the exactness.
                            waiting_time = abs(time_to_collisionPoint_i-time_to_collisionPoint_j);
                            % actual STAC
                            STAC_actual = max(time_to_collisionPoint_i,time_to_collisionPoint_j);
                            % adapted STAC
                            STAC_tmp = STAC_actual + waiting_time;                            
                        end

                        % store coupling information
                        coupling_infos(count).STAC = STAC_tmp;
                        coupling_infos(count).weight = weighting_function(STAC_tmp);
                        if is_leader
                            % vehicle_i is the leader
                            STAC(veh_i,veh_j) = STAC_tmp;
%                                 adjacency(veh_i,veh_j) = 1;
                            coupling_weights(veh_i,veh_j) = coupling_infos(count).weight; 
                            coupling_infos(count).idLeader = veh_i; 
                            coupling_infos(count).speedLeader = speed_i;
                            coupling_infos(count).positionLeader = position_i;
                            coupling_infos(count).idFollower = veh_j; 
                            coupling_infos(count).speedFollower = speed_j;
                            coupling_infos(count).positionFollower = position_j;
                        else
                            % vehicle_j is the leader
                            STAC(veh_j,veh_i) = STAC_tmp;
%                                 adjacency(veh_j,veh_i) = 1;
                            coupling_weights(veh_j,veh_i) = coupling_infos(count).weight; 
                            coupling_infos(count).idLeader = veh_j;
                            coupling_infos(count).speedLeader = speed_j;
                            coupling_infos(count).positionLeader = position_j;
                            coupling_infos(count).idFollower = veh_i;
                            coupling_infos(count).speedFollower = speed_i;
                            coupling_infos(count).positionFollower = position_i;
                        end
                        count = count + 1;
                        stop_flag = true;
                        break
                            
                    end
                    if stop_flag
                        break
                    end
                end   

                    
            end   
        end

    end
    % directed_adjacency(i,j)=1 indicates vehicle_i is coupled with vehicle_j and vehicle_i has a higher priority
    directed_adjacency = (coupling_weights ~= 0); 
    adjacency = triu(directed_adjacency,1) + triu(directed_adjacency',1); % undirected adjacency (symmetric)
    scenario.adjacency(:,:,scenario.k) = adjacency;

    scenario.coupling_weights = coupling_weights;
    scenario.coupling_infos = coupling_infos;  
end


%% local function
function idx_lanelet_pair = find_idx_lanelet_pair(predicted_lanelet_i, predicted_lanelet_j, lanelet_relationships)
    % First try to find predicted_lanelet_i in the field `ID_1` and
    % predicted_lanelet_j in the field `ID_2`. If not find, do it again in
    % the opposite way.
    idx_lanelets_i = find([lanelet_relationships.ID_1]==predicted_lanelet_i);
    idx_lanelets_j = find([lanelet_relationships.ID_2]==predicted_lanelet_j);
    idx_lanelet_pair = intersect(idx_lanelets_i,idx_lanelets_j);
    if isempty(idx_lanelet_pair)
        idx_lanelets_i = find([lanelet_relationships.ID_2]==predicted_lanelet_i);
        idx_lanelets_j = find([lanelet_relationships.ID_1]==predicted_lanelet_j);
        idx_lanelet_pair = intersect(idx_lanelets_i,idx_lanelets_j);
        if isempty(idx_lanelet_pair)
            % if still not find
            idx_lanelet_pair = [];
        end
    end
end

 
%% local function
function stop_point = get_stop_point_after_travel_certain_distance(position, lanelet, distance_to_travel)
% returns the stop point of the vehicle after travel a certain distance
% along the lanelet
    distances_to_veh = sqrt(sum((lanelet-position).^2,2));
    [~,idx_closest_to_veh] = min(distances_to_veh);
    distances_to_veh(1:idx_closest_to_veh) = inf; % points before vehicle's position are irrelevant
    distances_diff = distance_to_travel - distances_to_veh;
    [~,idx_colsest_to_stop_point] = min(abs(distances_diff));
    vector_direction = lanelet(idx_colsest_to_stop_point,:) - lanelet(idx_colsest_to_stop_point-1,:);
    unit_vector_direction = vector_direction/norm(vector_direction);
    stop_point = lanelet(idx_colsest_to_stop_point,:) + distances_diff(idx_colsest_to_stop_point)*unit_vector_direction;
end
%% local function
function coupling_weight = weighting_function(STAC,sensitive_factor)
    % returns the coupling weight based on the shortest time to achieve a
    % collision (STAC)
    if nargin==1
        sensitive_factor = 1; % sensitive factor used in calculating the coupling weights. The bigger, the more sensitive to the STAC. Default value 1.
    end
    coupling_weight = exp(-sensitive_factor*STAC); % negative exponential function
end

