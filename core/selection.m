function options = selection
    %% Choose scenario type
    types = {'Circle'};

    if ~exist('scenario_selection','var')
            scenario_selection = 1;
    end

    [scenario_selection,ok] = listdlg(...
            'ListString', types, ...
            'SelectionMode', 'single', ...
            'InitialValue', scenario_selection, ...
            'PromptString', 'Choose a scenario');

    if(~ok)
        error('Canceled');
    end

    %% Specify amount of vehicles
    scenarios = {
        '1', pi; ...
        '2', pi*(1:2); ...
        '3', 2*pi/3*(1:3); ...
        '4', 2*pi/4*(1:4); ...
        '5', 2*pi/5*(1:5); ...
        '6', 2*pi/6*(1:6); ...
        '7', 2*pi/7*(1:7); ...
        '8', 2*pi/8*(1:8); ...
        };

    if ~exist('vehicle_amount','var')
        vehicle_amount = 1;
    end

    [vehicle_amount,ok] = listdlg(...
        'ListString',{scenarios{:,1}}, ...
        'SelectionMode', 'single', ...
        'InitialValue', vehicle_amount, ...
        'PromptString', 'Choose the amount of vehicles');

    if(~ok)
        error('Canceled');
    end
    
    options.angles = scenarios{(scenario_selection - 1) * 8 + vehicle_amount,2};
    options.amount = vehicle_amount;
end

