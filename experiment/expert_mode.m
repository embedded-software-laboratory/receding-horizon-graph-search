function expert_mode(manual_vehicle_id, force_feedback_enabled)

    % Initialize data readers/writers...
    % getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    common_cpm_functions_path = fullfile( ...
        getenv('HOME'), 'dev/software/high_level_controller/examples/matlab' ...
    );
    assert(isfolder(common_cpm_functions_path), 'Missing folder "%s".', common_cpm_functions_path);
    addpath(common_cpm_functions_path);

    matlabDomainId = 1;
    [matlabParticipant,reader_vehicleStateList,~,~,reader_systemTrigger,writer_readyStatus,trigger_stop,writer_vehicleCommandDirect] = init_script(matlabDomainId); % #ok<ASGLU>

    % Set reader properties
    %reader_vehicleStateList.WaitSet = true;
    %reader_vehicleStateList.WaitSetTimeout = 5; % [s]

    % Middleware period for valid_after stamp
    dt_period_nanos = uint64(0.2*1e9);
    
    %ready_msg = ReadyStatus;
    %ready_msg.source_id = strcat('hlc_', num2str(1));
    %ready_stamp = TimeStamp;
    %ready_stamp.nanoseconds = uint64(0);
    %ready_msg.next_start_stamp = ready_stamp;
    %writer_readyStatus.write(ready_msg);

    % create subscriber
    wheelNode = ros2node("/wheel");
    wheelSub = ros2subscriber(wheelNode,"/j0","sensor_msgs/Joy");
    pause(0.2);

    if force_feedback_enabled
        g29_handler = G29ForceFeedback();
        g29_last_position = 0.0;
    end
    
    %[sample,~,~,~] = reader_vehicleStateList.take();
    %initial_message = wheelSub.LatestMessage;
    %timestamp_seC = uint64(initial_message.header.stamp.sec);
    %timestamp_nanoseC = uint64(initial_message.header.stamp.nanosec);
    %initial_timestamp = uint64(timestamp_seC*1e9 + timestamp_nanoseC);

    t_start = tic;

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
        
        %timestamp = timestamp_sec + timestamp_nanosec*1e-19;
        %timestamp = uint64(wheel_message.header.stamp.sec*1e10);
        %disp(timestamp);
        %timestamp_string = num2str(timestamp);
        %timestamp_string = strcat(timestamp_string,'000000001');
        %timestamp = uint64(str2num(timestamp_string));
        %timestamp = uint64(bitshift(timestamp, 9));
        %disp(timestamp);
    
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

        wheelData.steering = 1.5 * wheelData.steering;

        if throttle > 0.5
            throttle = 0.5;
        elseif throttle < -0.5
            throttle = -0.5;
        end
        disp(sprintf("throttle: %f, steering: %f", throttle, wheelData.steering));
        vehicle_command_direct.motor_throttle = double(throttle);
        vehicle_command_direct.steering_servo = double(wheelData.steering);

        %{
        format long
        timestamp_sec = uint64(wheel_message.header.stamp.sec);
        timestamp_nanosec = uint64(wheel_message.header.stamp.nanosec);
        timestamp_str_sec = num2str(timestamp_sec);
        timestamp_str_nanosec = num2str(timestamp_nanosec);
        timestamp_string = strcat(timestamp_str_sec,timestamp_str_nanosec);
        timestamp = uint64(str2num(timestamp_string));
        %}

        timestamp_sec = uint64(wheel_message.header.stamp.sec);
        timestamp_nanosec = uint64(wheel_message.header.stamp.nanosec);
        timestamp = uint64(timestamp_sec*1e9 + timestamp_nanosec);
        %disp(timestamp);

        %if timestamp > initial_timestamp + dt_period_nanos
            %sample(end).t_now = sample(end).t_now + dt_period_nanos;
            %initial_timestamp = sample(end).t_now;
        %end

        t = toc(t_start);

        %if t*1e9 > dt_period_nanos - threshold
        if t > 0.01
            %sample(end).t_now = sample(end).t_now + dt_period_nanos;
            %sample(end).t_now = sample(end).t_now + 0.2*1e9;
            t_start = tic;
            %threshold = t*1e9 - dt_period_nanos;
            %threshold = t*1e9 - 0.2*1e8;

            vehicle_command_direct.header.create_stamp.nanoseconds = ...
                uint64(timestamp);
            vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                uint64(timestamp+uint64(4*1e7));
            %vehicle_command_direct.header.create_stamp.nanoseconds = ...
                %uint64(sample(end).t_now);
            %vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
                %uint64(sample(end).t_now+(0.2*1e8));
            writer_vehicleCommandDirect.write(vehicle_command_direct);

        end

        %disp(sample(end).t_now);

        %{
        vehicle_command_direct.header.create_stamp.nanoseconds = ...
            uint64(timestamp);
        vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
            uint64(timestamp+dt_period_nanos);
        %vehicle_command_direct.header.create_stamp.nanoseconds = ...
            %uint64(sample(end).t_now);
        %vehicle_command_direct.header.valid_after_stamp.nanoseconds = ...
            %uint64(sample(end).t_now+(0.2*1e8));
        writer_vehicleCommandDirect.write(vehicle_command_direct);
        %}
    
        if force_feedback_enabled
            g29_last_position = g29_handler.g29_send_message(0.0, 0.5, g29_last_position);
        end
    end
    
end
