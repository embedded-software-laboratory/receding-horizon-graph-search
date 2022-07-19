function scenario = communication_init(scenario, exp)
% COMMUNICATION_INIT This function initializes the communication network.
% ROS 2 nodes are created for each vehicle. Each vehicle has its own topic
% and sends its data only to its own topic.
% 
% INPUT:
%   scenario: instance of the class Scenario
%   
%   exp: object of the class 'SimLab' or 'CPMLab'
% 
% OUTPUT:
%   scenario: instance of the class Scenario with instance of the class
%   Communication added to it

    % generate custom message type (for vehicle communication) if not exist
    msgList = ros2("msg","list"); % get all ROS 2 message types
    [file_path,~,~] = fileparts(mfilename('fullpath'));
    if sum(cellfun(@(c)strcmp(c,'veh_msgs/Traffic'), msgList))==0
        % if the message type 'veh_msgs/Traffic' does not exist 
        path_custom_msg = [file_path,filesep,'cust1'];
        
        % Generate custom messages. NOTE that Python, CMake, and a C++ compiler are required (see
        % https://de.mathworks.com/help/ros/gs/ros-system-requirements.html
        % for more details according to your own MATLAB version).
        %
        % Useful functions: 
        % 1. pyenv % check python version used by MATLAB
        % 2. pyenv('Version','requiredPythonVersionNumber') or pyenv('Version','fullPathOfPythonExe')
        % 3. !cmake --version % CMake version 
        % 4. mex -setup % set c language compiler
        %
        % Note that sometimes ros2genmsg fails although all denpendencies
        % exist because the path where the custom messages are stored is
        % too deep. Try to move them to shallower path and try again.
        disp('Generating ROS 2 custom message type...')
        try
            ros2genmsg(path_custom_msg)
        catch ME
            disp(['If all environments for ros2genmsg() are prepared but still failed, try to move the whole folder to a ' ...
                'shallower path and run again if you use Windows machine, which sadly has a max path limit constraint.'])
            throw(ME)
        end
    end

    if sum(cellfun(@(c)strcmp(c,'ros_g29_force_feedback/ForceFeedback'), msgList))==0
        if ~scenario.options.is_sim_lab    
            if ispc
                error('You are using a Windows machine, please do not select lab mode!')
            end
            % This message type is only needed for lab mode but not
            % simulation mode
            path_custom_msg = [file_path,filesep,'cust2'];
            disp('Generating ROS 2 custom message type...')
            ros2genmsg(path_custom_msg)
        end
    end

    nVeh = scenario.nVeh;
    Hp = scenario.Hp;

    % measure vehicles' initial poses and trims
    [x0_measured, trims_measured] = exp.measure(false);

    start = tic; 
    disp('Creating ROS 2 publishers...')
    for iVeh = 1:nVeh
        scenario.vehicles(iVeh).communicate = Communication(); % create instance of the Comunication class
        scenario.vehicles(iVeh).communicate = initialize_communication(scenario.vehicles(iVeh).communicate, scenario.vehicles(iVeh).ID); % initialize
        scenario.vehicles(iVeh).communicate = create_publisher(scenario.vehicles(iVeh).communicate); % create publisher
    end
    
    % Create subscribers.
    % Each vehicle subscribes all other vehicles.
    % NOTE that subscribers are create only once but not loopover all
    % vehicles to let all of them subscribe others because it is
    % time-consuming to create many subscribers. 
    % The subscribers will be used by all vehicles.
    disp('Creating ROS 2 subscribers...')
    vehs_to_be_subscribed = [scenario.vehicles.ID];
    scenario.ros_subscribers = create_subscriber(scenario.vehicles(1).communicate,vehs_to_be_subscribed);

    duration = toc(start);
    disp(['Finished in ' num2str(duration) ' seconds.'])

    if ~scenario.options.is_mixed_traffic
        % Communicate predicted trims, pridicted lanelets and areas to other vehicles
        for jVeh = 1:nVeh
            predicted_trims = repmat(trims_measured(jVeh), 1, Hp+1); % current trim and predicted trims in the prediction horizon

            x0 = x0_measured(jVeh,indices().x); % vehicle position x
            y0 = x0_measured(jVeh,indices().y); % vehicle position y

            predicted_lanelets = get_predicted_lanelets(scenario,jVeh,predicted_trims(1),x0,y0);

            predicted_occupied_areas = {}; % for initial time step, the occupied areas are not predicted yet
            scenario.vehicles(jVeh).communicate.send_message(scenario.k, predicted_trims, predicted_lanelets, predicted_occupied_areas);   
        end
    end
end
