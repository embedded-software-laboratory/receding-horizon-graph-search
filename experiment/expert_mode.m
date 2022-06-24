function expert_mode(manual_vehicle_id)

    % Initialize data readers/writers...
    % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    common_cpm_functions_path = fullfile( ...
        getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    );
    assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
    addpath(common_cpm_functions_path);

    matlabDomainId = 1;
    [~,reader_vehicleStateList,~, ~, ~,writer_readyStatus,~,writer_vehicleCommandDirect] = init_script(matlabDomainId); % #ok<ASGLU>

    % Set reader properties
    reader_vehicleStateList.WaitSet = true;
    reader_vehicleStateList.WaitSetTimeout = 5; % [s]

    % Middleware period for valid_after stamp
    dt_period_nanos = uint64(200*1e9);
    
    ready_msg = ReadyStatus;
    ready_msg.source_id = strcat('hlc_', num2str(1));
    ready_stamp = TimeStamp;
    ready_stamp.nanoseconds = uint64(0);
    ready_msg.next_start_stamp = ready_stamp;
    writer_readyStatus.write(ready_msg);

    % create subscriber
    wheelNode = ros2node("/wheel");
    wheelSub = ros2subscriber(wheelNode,"/j0","sensor_msgs/Joy");
    pause(0.2);

    while(true)

        wheel_message = wheelSub.LatestMessage;

        wheelData = struct;
        wheelData.axes = wheel_message.axes;
        wheelData.buttons = wheel_message.buttons;
        wheelData.steering =  wheelData.axes(1);
        wheelData.throttle = wheelData.axes(3);
        wheelData.brake = wheelData.axes(4);
        wheelData.leftPaddle = wheelData.buttons(6);
        wheelData.rightPaddle = wheelData.buttons(5);
        
        %{
        if modeHandler.steering < -0.5
            modeHandler.steering = -0.5;
        elseif modeHandler.steering > 0.5
            modeHandler.steering = 0.5;
        end

        modeHandler.steering = 2.0 * modeHandler.steering;

        
        % make steering back to 0 easier
        if modeHandler.steering < 0 && obj.lastSteeringValue < modeHandler.steering
            modeHandler.steering = modeHandler.steering / 1.5;
        elseif modeHandler.steering > 0 && obj.lastSteeringValue > modeHandler.steering
            modeHandler.steering = modeHandler.steering / 1.5;
        end
        %}
    
        [sample,~,~,~] = reader_vehicleStateList.take();

        dt_max_comm_delay = uint64(100e6);
        if dt_period_nanos >= dt_max_comm_delay
            dt_valid_after = obj.dt_period_nanos;
        else
            dt_valid_after = dt_max_comm_delay;
        end

        vehicle_command_direct = VehicleCommandDirect;
        vehicle_command_direct.vehicle_id = uint8(manual_vehicle_id);

        throttle = 0;
        if wheelData.throttle >= 0
            throttle = wheelData.throttle;
        elseif wheelData.brake >= 0
            throttle = (-1) * wheelData.brake;
        end
        %{
        if wheelData.steering < -0.4
            wheelData.steering = -0.4;
        elseif wheelData.steering > 0.4
            wheelData.steering = 0.4;
        end
        %}
        wheelData.steering = 2.0 * wheelData.steering;

        if throttle > 0.4
            throttle = 0.4;
        elseif throttle < -0.4
            throttle = -0.4;
        end
        disp(sprintf("throttle: %f, steering: %f", throttle, wheelData.steering));
        vehicle_command_direct.motor_throttle = double(throttle);
        vehicle_command_direct.steering_servo = double(wheelData.steering);
        %vehicle_command_direct.header.create_stamp.nanoseconds = ...
            %uint64(timestamp);
        %vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
            %uint64(timestamp) + uint64(dt_valid_after);
        vehicle_command_direct.header.create_stamp.nanoseconds = ...
            uint64(sample.t_now);
        vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
            uint64(sample.t_now+dt_valid_after);
        writer_vehicleCommandDirect.write(vehicle_command_direct);
    end
    
end
