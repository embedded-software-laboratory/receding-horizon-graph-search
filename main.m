%% Add modules to path
% Import tree class
matlab_tree = './matlab-tree/';
assert(logical(exist(matlab_tree, 'dir')));
addpath(matlab_tree);

addpath(genpath(pwd));

warning('off','MATLAB:polyshape:repairedBySimplify')

prompt = {'Enter Scenario:'};
dlgtitle = 'Input';

answer = inputdlg(prompt);

while(~isempty(answer))

    if(answer{1} == '1')
        test1;
        answer = inputdlg(prompt);
    elseif (answer{1} == '2') 
        test2;
        answer = inputdlg(prompt);
    elseif (answer{1} == '3')
        test3;
        answer = inputdlg(prompt);
    else
        %% Define scenario
        % Obstacles

        % Vehicles
        nVeh = 3;
        scenario = Scenario(2*pi/nVeh*(1:nVeh));

        % Initial position
        init_poses = [];
        trim_indices = 5 * ones(1, nVeh);

        for i = 1:nVeh
            init_pose.x = scenario.vehicles(i).x_start;
            init_pose.y = scenario.vehicles(i).y_start;
            init_pose.yaw = scenario.vehicles(i).heading;
            init_poses = [init_poses, init_pose];
        end



        % Goal
        target_poses = [];
        step = floor(nVeh / 2);

        for i = 1:nVeh

            k = mod(i + step, nVeh) + 1;

            target_pose.x = scenario.vehicles(k).x_start;
            target_pose.y = scenario.vehicles(k).y_start;
            target_pose.yaw = scenario.vehicles(k).heading;
            target_poses = [target_poses, target_pose];
        end

        %% Define motion graph
        % Choose Model
        model = BicycleModel(2.2,2.2);

        % Primitive duration
        primitive_dt = 1;

        % Trims
        velocity = 3;

        % trims left to right
        load('trim_inputs_6');
        load('trim_adjacency_6_1');
        n_trims = length(u_trims);

        % Generate graph motion graphs
        motionGraphList = [];
        for i = 1:nVeh

           motionGraph = MotionGraph(model, u_trims, trim_adjacency, primitive_dt); 
           motionGraphList = [motionGraphList, motionGraph];

        end

        % Combine graphs
        combinedGraph = CombinedGraph(motionGraphList);
        %% Graph search
        % Choose search algorithm
        depth = 20;
        fig = figure('Name','Trajectories','NumberTitle','off');
        axis_size = [-30 30 -30 30];
        search_tree = generate_tree(init_poses, target_poses, trim_indices, combinedGraph, depth, @is_collision, @get_next_node_astar, fig, axis_size);

        % Search
        search_paths = return_path(search_tree, combinedGraph);

        %% Visualize
        plot(scenario, fig);
        vis_trajectory(search_paths, target_poses, axis_size, fig);

        figure('Name','Search Tree','NumberTitle','off');
        plot(search_tree);

        answer = inputdlg(prompt);
    end
end