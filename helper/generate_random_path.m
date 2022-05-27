function [random_path, scenario] = generate_random_path(scenario, vehid, n, startPosition)
    % GENERATE_RANDOM_PATH    returns a ref_path struct
    % random_path.lanelets_index: lanelet index of the reference path
    % random_path.path: reference path including x and y information
    % random_path.points_index: count the max index of reference points for each lanelets

    random_path = struct;
    
    disp(sprintf('Id: %d', vehid));
    % maybe scenario.vehicles not yet initialized
    % lanelets_index_vehid = scenario.vehicles(1,vehid).lanelets_index; 
    lanelets_index_vehid = startPosition;  
    
    load('commonroad_data.mat');
    commonroad = commonroad_data;

    %{ 
    if vehicle position not directly accessable, iterate over lanelets to find position
    for i = 1:length(lanelets)
        if    
    end
    %}

    % find initial position for this
    random_path.lanelets_index = [lanelets_index_vehid];
    laneChangeAllowed = false;

    while length(random_path.lanelets_index) < n
        % find all edges that are the successor of the current lane or adjacent and lane change enabled
        subsequent_lanes = struct;
        subsequent_lanes.lanelets_index = [0];

        for i = 1:length(commonroad_data.lanelet)
            
            % find id of current lane
            if random_path.lanelets_index(end) == commonroad_data.lanelet(i).idAttribute
                successor = commonroad_data.lanelet(i).successor;
                successor_indices = horzcat(successor.refAttribute);
                subsequent_indices = successor_indices;


                if laneChangeAllowed
                    %disp(subsequent_indices);
                    for k = 1:length(successor_indices)
                        successor_adjacentLeft = commonroad_data.lanelet(successor_indices(k)).adjacentLeft;
                        if isfield(successor_adjacentLeft,'refAttribute') && strcmp(successor_adjacentLeft.drivingDirAttribute,'same')
                            subsequent_indices = horzcat(subsequent_indices, successor_adjacentLeft.refAttribute);
                            %disp(subsequent_indices);
                        end

                        successor_adjacentRight = commonroad_data.lanelet(successor_indices(k)).adjacentRight;
                        if isfield(successor_adjacentRight,'refAttribute') && strcmp(successor_adjacentRight.drivingDirAttribute,'same')
                            subsequent_indices = horzcat(subsequent_indices, successor_adjacentRight.refAttribute);
                            %disp(subsequent_indices);
                        end

                        % include check for predecessor of successor
                    end
                end
                
                for j = 1:length(subsequent_indices)
                    if subsequent_lanes.lanelets_index(1) == 0
                        subsequent_lanes.lanelets_index(1) = subsequent_indices(j);
                    else
                        subsequent_lanes.lanelets_index(end+1) = subsequent_indices(j);
                    end
                end

                %disp(sprintf('subsequent lanelets index size: %d', length(subsequent_lanes.lanelets_index)));
            end
        end
        
        if laneChangeAllowed
            range = length(subsequent_lanes.lanelets_index);
            index = randi(range);
            %disp(sprintf('range: %d, index: %d', range, index));

            % TODO: find better way to delete lanes from reference path
            % select successor of current lane as start for reference path
            if random_path.lanelets_index(1) == lanelets_index_vehid
                random_path.lanelets_index(end) = subsequent_lanes.lanelets_index(index);
            else
                random_path.lanelets_index(end+1) = subsequent_lanes.lanelets_index(index);
            end
            
            
            %random_path.lanelets_index(end+1) = subsequent_lanes.lanelets_index(index);

            %disp(sprintf('random at end: %d', random_path.lanelets_index(end)));
            laneChangeAllowed = false;
        else
            if length(subsequent_lanes.lanelets_index) > 1
                if (random_path.lanelets_index(end) == 3 || random_path.lanelets_index(end) == 5 || random_path.lanelets_index(end) == 81 ... 
                    || random_path.lanelets_index(end) == 83)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 2;
                elseif (random_path.lanelets_index(end) == 11 || random_path.lanelets_index(end) == 89)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 15;
                elseif (random_path.lanelets_index(end) == 12 || random_path.lanelets_index(end) == 40 || random_path.lanelets_index(end) == 66 ...
                        || random_path.lanelets_index(end) == 90)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 5;
                elseif (random_path.lanelets_index(end) == 22 || random_path.lanelets_index(end) == 100)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) - 17;
                elseif (random_path.lanelets_index(end) == 29 || random_path.lanelets_index(end) == 31 || random_path.lanelets_index(end) == 55 ...
                        || random_path.lanelets_index(end) == 57)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) - 2;
                elseif (random_path.lanelets_index(end) == 39 || random_path.lanelets_index(end) == 65)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) + 11;
                elseif (random_path.lanelets_index(end) == 49 || random_path.lanelets_index(end) == 75)
                    random_path.lanelets_index(end+1) = random_path.lanelets_index(end) - 20;
                end 
            else
                random_path.lanelets_index(end+1) = subsequent_lanes.lanelets_index(end);
            end

            laneChangeAllowed = true;
        end
    end

    %random_path.lanelets_index = [59,57,55,67,65,98,37,35,31,29,27,1,3,5,7];

    for i = 1:length(random_path.lanelets_index)
        disp(sprintf('random entries: i: %d, entry: %d', i,random_path.lanelets_index(i)));
    end

    [lanelets, ~,~,~,~,scenario.lanelet_boundary] = commonroad_lanelets(); 

    % reference path
    path = [];

    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1,length( random_path.lanelets_index));

    for nlanelets = 1:length(random_path.lanelets_index)
        % choose the center line of the lanelet as reference path
        randomPath_x = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx);
        randomPath_y = lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cy);

        if length(randomPath_x) > 3
            %randomPath_x = randomPath_x([2:(end)-1]);
        end
        
        if length(randomPath_y) > 3
            %randomPath_y = randomPath_y([2:(end)-1]);
        end

        randomPath_next = [randomPath_x(1:end),randomPath_y(1:end)];
        path = [path; randomPath_next];

        % count the number of points of each lanelets
        Npoints = length(randomPath_next);
        % max point index of each lanelet
        lanelet_point_max = lanelet_point_max + Npoints;
        points_index(nlanelets) = lanelet_point_max;

    end 
    random_path.path = path;
    random_path.points_index = points_index;

    %{
    % reduce lanelet boundarys to get same number of points as lanelets
    for index = 1:length(scenario.lanelet_boundary)
        scenario.lanelet_boundary{1,index}{1,1} = scenario.lanelet_boundary{1,index}{1,1}(2:(end)-1,1:2);
        scenario.lanelet_boundary{1,index}{1,2} = scenario.lanelet_boundary{1,index}{1,2}(2:(end)-1,1:2);
    end
    %}

    %{
    % the max points index of each lanelet
    lanelet_point_max = 0;
    points_index = zeros(1,length( random_path.lanelets_index));
    for nlanelets = 1:length( random_path.lanelets_index)
        % count the number of points of each lanelets
        Npoints = length(lanelets{ random_path.lanelets_index(nlanelets)}(:,LaneletInfo.cx));
        % max point index of each lanelet
        lanelet_point_max = lanelet_point_max + Npoints;
        points_index(nlanelets) = lanelet_point_max;

    end 
    random_path.points_index = points_index;
    %}

end