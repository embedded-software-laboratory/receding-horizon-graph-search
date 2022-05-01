function iter = rhc_init(scenario, x_measured, trim_indices, initialized_reference_path, mVehid, isPB)
% RHC_INIT  Preprocessing step for RHC controller

    idx = indices();
    if ~initialized_reference_path
        for iVeh = 1:scenario.nVeh
            index = match_pose_to_lane(x_measured(iVeh, idx.x), x_measured(iVeh, idx.y));
            disp(sprintf("veh ID: %d, index: %d", iVeh, index));

            if (mVehid == iVeh)
                updated_ref_path = generate_manual_path(scenario, iVeh, 20, index);     
            else
                % ref_path = generate_ref_path(vehid(iveh));% function to generate refpath based on CPM Lab road geometry
                updated_ref_path = generate_random_path(scenario, iVeh, 20, index); % function to generate random path for autonomous vehicles based on CPM Lab road geometry
            end
            
            updatedRefPath = updated_ref_path.path;
            scenario.vehicles(iVeh).x_start = updatedRefPath(1,1);
            scenario.vehicles(iVeh).y_start = updatedRefPath(1,2);
            scenario.vehicles(iVeh).x_goal = updatedRefPath(2:end,1);
            scenario.vehicles(iVeh).y_goal = updatedRefPath(2:end,2);
            
            scenario.vehicles(iVeh).referenceTrajectory = [scenario.vehicles(iVeh).x_start scenario.vehicles(iVeh).y_start
                                    scenario.vehicles(iVeh).x_goal  scenario.vehicles(iVeh).y_goal];
            scenario.vehicles(iVeh).lanelets_index = updated_ref_path.lanelets_index;
            scenario.vehicles(iVeh).points_index = updated_ref_path.points_index;

            yaw = calculate_yaw(updatedRefPath);
            scenario.vehicles(iVeh).yaw_start = yaw(1);
            scenario.vehicles(iVeh).yaw_goal = yaw(2:end); 
        end

        scenario.plot_limits = [0,4.5;0,4];  
        scenario.T_end = 60;
        scenario.model = BicycleModel(scenario.vehicles(end).Lf,scenario.vehicles(end).Lr); %why only single veh?
        nVeh_mpa = scenario.nVeh;
        scenario.Hp = 6;
        
        if isPB 
            scenario.adjacency = zeros(scenario.nVeh,scenario.nVeh);
            scenario.assignPrios = true;
            scenario.controller_name = strcat(scenario.controller_name, '-PB');
            scenario.controller = @(s,i) pb_controller(s,i);
            nVeh_mpa = 1;

        end    
        
        recursive_feasibility = true;
        scenario.mpa = MotionPrimitiveAutomaton(...
            scenario.model...
            , scenario.trim_set...
            , scenario.offset...
            , scenario.dt...
            , nVeh_mpa...
            , scenario.Hp...
            , scenario.tick_per_step...
            , recursive_feasibility...
        );
    end
    
    iter = struct;
    iter.referenceTrajectoryPoints = zeros(scenario.nVeh,scenario.Hp,2);
    iter.referenceTrajectoryIndex = zeros(scenario.nVeh,scenario.Hp,1);
    iter.x0 = x_measured;
    iter.trim_indices = trim_indices;
    
    iter.vRef = zeros(scenario.nVeh,scenario.Hp);
    for iVeh=1:scenario.nVeh
        iter.vRef(iVeh,:) = get_max_speed(scenario.mpa,trim_indices(iVeh));
        % Find equidistant points on the reference trajectory.
        reference = sampleReferenceTrajectory(...
            scenario.Hp, ... % number of prediction steps
            scenario.vehicles(iVeh).referenceTrajectory, ...
            iter.x0(iVeh,idx.x), ... % vehicle position x
            iter.x0(iVeh,idx.y), ... % vehicle position y
            iter.vRef(iVeh,:)*scenario.dt...  % distance traveled in one timestep
        );
    
        iter.referenceTrajectoryPoints(iVeh,:,:) = reference.ReferencePoints;
        iter.referenceTrajectoryIndex(iVeh,:,:) = reference.ReferenceIndex;
    
    end
    
   
    % Determine Obstacle positions (x = x0 + v*t)
    % iter.obstacleFutureTrajectories = zeros(scenario.nObst,2,scenario.Hp);
    % for k=1:scenario.Hp
    %     step = (k*scenario.dt+scenario.delay_x + scenario.dt + scenario.delay_u)*scenario.obstacles(:,idx.speed);
    %     iter.obstacleFutureTrajectories(:,idx.x,k) = step.*cos( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.x);
    %     iter.obstacleFutureTrajectories(:,idx.y,k) = step.*sin( scenario.obstacles(:,idx.heading) ) + obstacleState(:,idx.y);
    % end
    
    % iter.uMax = uMax;

end