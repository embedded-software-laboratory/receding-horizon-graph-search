function result = run_experiment(varargin)
% RUN_EXPERIMENT    Runtime function for usage in the cpm lab.

%% Setup

%% Read input
vehicle_ids = [varargin{1:end-1}];
assert(issorted(vehicle_ids));

%% Initialize data readers/writers...
% getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
common_cpm_functions_path = fullfile( ...
    '../examples/matlab' ...
);
assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
addpath(common_cpm_functions_path);

matlabDomainId = 1;
[matlabParticipant, reader_vehicleStateList, writer_vehicleCommandTrajectory, ~, reader_systemTrigger, writer_readyStatus, trigger_stop] = init_script(matlabDomainId); %#ok<ASGLU>

% Set reader properties
reader_vehicleStateList.WaitSet = true;
reader_vehicleStateList.WaitSetTimeout = 5; % [s]

%% Setup the HLC

% Setup scenario
scenario = varargin{end};

% Setup controller
info = struct;
info.trim_indices = [scenario.vehicles(:).trim_config];
% Initialize
cur_depth = 1;

controller = @(scenario, iter)...
    graph_search(scenario, iter);


% init result struct
result = get_result_struct(scenario);
controller_init = false;

% Middleware period for valid_after stamp
dt_period_nanos = uint64(scenario.dt*1e9);


%% Sync start with infrastructure
% Send ready signal for all assigned vehicle ids
disp('Sending ready signal');
for iVehicle = vehicle_ids
    ready_msg = ReadyStatus;
    ready_msg.source_id = strcat('hlc_', num2str(iVehicle));
    ready_stamp = TimeStamp;
    ready_stamp.nanoseconds = uint64(0);
    ready_msg.next_start_stamp = ready_stamp;
    writer_readyStatus.write(ready_msg);
end

% Wait for start or stop signal
disp('Waiting for start or stop signal');    
got_stop = false;
got_start = false;
while (~got_stop && ~got_start)
    [got_start, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
end

%% Main control loop
while (~got_stop)
    % Measurement
    % -------------------------------------------------------------------------
    [sample, ~, sample_count, ~] = reader_vehicleStateList.take();
    if (sample_count > 1)
        warning('Received %d samples, expected 1. Correct middleware period? Missed deadline?', sample_count);
    end
    
    
    % Control 
    % -------------------------------------------------------------------------
    % one-time initialization of starting position
    if controller_init == false
        x0 = zeros(scenario.nVeh,4);
        pose = [sample(end).state_list.pose];
        x0(:,1) = [pose.x];
        x0(:,2) = [pose.y];
        x0(:,3) = [pose.yaw];
        x0(:,4) = [sample(end).state_list.speed];
        controller_init = true;
    else
        % take last planned state as new actual state
        planned_node = info.Tree.node{info.tree_path(2)};
        speeds = zeros(scenario.nVeh,1);
        for iVeh=1:scenario.nVeh
            speeds(iVeh) = scenario.mpa.trims(planned_node(iVeh,NodeInfo.trim)).speed;
        end
        x0 = [planned_node(:,NodeInfo.x), planned_node(:,NodeInfo.y), planned_node(:,NodeInfo.yaw), speeds];
    end
    % Sample reference trajectory
    iter = rhc_init(scenario,x0,info.trim_indices);
    result.iteration_structs{cur_depth} = iter;
    controller_timer = tic;
        [u, y_pred, info] = controller(scenario, iter);
    result.controller_runtime(cur_depth) = toc(controller_timer);
    % save controller outputs in resultstruct
    result.trajectory_predictions(:,cur_depth) = y_pred;
    result.controller_outputs{cur_depth} = u;
    % store vehicles path in higher resolution
    result.vehicle_path_fullres(:,cur_depth) = path_between(...
        info.Tree.node{info.tree_path(1)}...
        ,info.Tree.node{info.tree_path(2)}...
        ,info.Tree...
        ,scenario.mpa...
    );
            
    result.n_expanded = result.n_expanded + numel(info.Tree.node);
            
    % Apply control action f/e veh
    % -------------------------------------------------------------------------
    out_of_map_limits = false(scenario.nVeh,1);
    for iVeh = 1:scenario.nVeh
        n_traj_pts = numel(info.tree_path)-1;
        n_predicted_points = size(y_pred{iVeh},1);
        idx_predicted_points = 1:n_predicted_points/n_traj_pts:n_predicted_points;
        trajectory_points(1:n_traj_pts) = TrajectoryPoint;
        for i_traj_pt = 1:n_traj_pts
            i_predicted_points = idx_predicted_points(i_traj_pt);
            trajectory_points(i_traj_pt).t.nanoseconds = ...
                uint64(sample(end).t_now + i_traj_pt*dt_period_nanos);
            trajectory_points(i_traj_pt).px = y_pred{iVeh}(i_predicted_points,1);
            trajectory_points(i_traj_pt).py = y_pred{iVeh}(i_predicted_points,2);
            yaw = y_pred{iVeh}(i_predicted_points,3);
            speed = scenario.mpa.trims(y_pred{iVeh}(i_predicted_points,4)).speed;
            trajectory_points(i_traj_pt).vx = cos(yaw)*speed;
            trajectory_points(i_traj_pt).vy = sin(yaw)*speed;
        end
        out_of_map_limits(iVeh) = is_veh_at_map_border(trajectory_points);
        vehicle_command_trajectory = VehicleCommandTrajectory;
        vehicle_command_trajectory.vehicle_id = uint8(vehicle_ids(iVeh));
        vehicle_command_trajectory.trajectory_points = trajectory_points;
        vehicle_command_trajectory.header.create_stamp.nanoseconds = ...
            uint64(sample(end).t_now);
        vehicle_command_trajectory.header.valid_after_stamp.nanoseconds = ...
            uint64(sample(end).t_now + dt_period_nanos);
        writer_vehicleCommandTrajectory.write(vehicle_command_trajectory);
    end



    % Check for stop signal
    % -------------------------------------------------------------------------
    [~, got_stop] = read_system_trigger(reader_systemTrigger, trigger_stop);
    if any(out_of_map_limits)
        got_stop = true;
    end


    cur_depth = cur_depth + 1;
end


%% save results
save(fullfile(result.output_path,'data.mat'),'result');


end


function stop_experiment = is_veh_at_map_border(trajectory_points)
    % Vehicle command timeout is 1000 ms after the last valid_after_stamp,
    % so vehicle initiates stop between third and fourth trajectory point
    vhlength = 0.25;
    x_min =  vhlength + 0;
    x_max = -vhlength + 4.5;
    y_min =  vhlength + 0;
    y_max = -vhlength + 4.0;
    px = trajectory_points(4).px;
    py = trajectory_points(4).py;
    stop_experiment = x_min>px || px>x_max ...
                   || y_min>py || py>y_max;
end
