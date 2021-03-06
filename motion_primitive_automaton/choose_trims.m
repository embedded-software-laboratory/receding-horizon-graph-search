function [trim_inputs, trim_adjacency] = choose_trims(trim_set)
% CHOOSE_TRIMS  Choose trim set used.

    switch trim_set
    case 1
        %% Three speeds (0 to 1.5 m/s), one steering (0 deg)
        speeds = 0:0.75:1.5;
        ntrims = numel(speeds);
        trim_inputs = zeros(ntrims,2);
        trim_inputs(:,2) = speeds;
        trim_adjacency = eye(ntrims);
        trim_adjacency(1:end-1,2:end) = trim_adjacency(1:end-1,2:end) ...
            + eye(ntrims-1);
        trim_adjacency(2:end,1:end-1) = trim_adjacency(2:end,1:end-1) ...
            + eye(ntrims-1);
    case 3
        %% Two speeds (0 and 0.75 m/s), three steering
        steering = (-1:1) * pi/9;
        ntrims = numel(steering)+1;
        trim_inputs = zeros(ntrims,2);
        trim_inputs(2:end,1) = steering;
        trim_inputs(2:end,2) = 0.75;%10 / 3.6;
        % All states connected
        trim_adjacency = ones(ntrims);
    case 5
        %% Two speeds (0 and 0.75 m/s), five steering
        steering = (-2:2) * pi/18;
        ntrims = numel(steering)+1;
        trim_inputs = zeros(ntrims,2);
        trim_inputs(2:end,1) = steering;
        trim_inputs(2:end,2) = 0.75;%10 / 3.6;
        trim_adjacency = ones(ntrims);
        % equilibrium is state 1
        % equilibrium is reachable from all states
        % and can reach all states, so all 1s is good
        % other states are connected by one hop
        trim_adjacency(2:end,2:end) = trim_adjacency(2:end,2:end) ...
            - triu(ones(ntrims-1),2)...
            - tril(ones(ntrims-1),-2);
    end
end