function options = selection(vehicle_amount_p,plot_option_p,strat_option_p, scenario,priority_assignment)
% SELECTION     Choose scenario/simulation properties in user interface.

%% Specify amount of vehicles
scenarios = {
    '1', pi+pi; ...
    '2', pi+pi*(1:2); ...
    '3', pi+2*pi/3*(1:3); ...
    '4', pi+2*pi/4*(1:4); ...
    '5', pi+2*pi/5*(1:5); ...
    '6', pi+2*pi/6*(1:6); ...
    '7', pi+2*pi/7*(1:7); ...
    '8', pi+2*pi/8*(1:8); ...
    '9', pi+2*pi/9*(1:10); ...
    '10', pi+2*pi/10*(1:10); ...
    '11', pi+2*pi/11*(1:11); ...
    '12', pi+2*pi/12*(1:12); ...
    '13', pi+2*pi/13*(1:13); ...
    '14', pi+2*pi/14*(1:14); ...
    '15', pi+2*pi/15*(1:15); ...
    '16', pi+2*pi/15*(1:16); ...
    '17', pi+2*pi/15*(1:17); ...
    '18', pi+2*pi/15*(1:18); ...
    '19', pi+2*pi/15*(1:19); ...
    '20', pi+2*pi/15*(1:20); ...
    '21', pi+2*pi/15*(1:21); ...
    '22', pi+2*pi/15*(1:22); ...
    '23', pi+2*pi/15*(1:23); ...
    '24', pi+2*pi/15*(1:24); ...
    '25', pi+2*pi/15*(1:25); ...
    };
scenarioName = {
    '1','Circle_scenario';...
    '2','Commonroad'...
    };
priorityAssignment = {
    '1','topo_priority';...
    '2','right_of_way_priority'
    '2','constant_priority';...
    '3','random_priority';...
    '4','FCA_priority'
    };

possPlots = {
    '1', 'no visualization',                    [false,false]; ...
    '2', 'vehicle visualization',               [true,false]; ...
    '3', 'visualization + node exploration',    [true,true]; ... % only for centralized controller
    };

possStrats = {
    '1', 'centralized'; ...
    '2', 'pb non-coop'; ...
    };



    % ====== load previous choice ======
    try
        load([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option', 'strat_option','scenario_option','priority_option');
    catch
        % continue
    end

if nargin < 1
    if ~exist('vehicle_amount','var')
        vehicle_amount = 1;
    end
    if ~exist('plot_option','var') || plot_option == 3
        plot_option = 2;
    end
    if ~exist('strat_option','var')
        strat_option = 1;
    end
    if ~exist('scenario_option','var')
        scenario_option = 1;
    end
    if ~exist('priority_option','var')
        priority_option = 1;
    end

    [scenario_option,ok] = listdlg(...
        'ListString',scenarioName (:,2), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', scenario_option, ...
        'PromptString', 'Choose the scenario');    
    
    if(~ok)
        error('Canceled');
    end
    
    [strat_option,ok] = listdlg(...
        'ListString',possStrats(:,2), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', strat_option, ...
        'PromptString', 'Choose the control strategy');

    if(~ok)
        error('Canceled');
    end
    
    [priority_option,ok] = listdlg(...
        'ListString',priorityAssignment(:,2), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', priority_option, ...
        'PromptString', 'Choose the priority assignment method');    
    
    if(~ok)
        error('Canceled');
    end    
    
    [vehicle_amount,ok] = listdlg(...
        'ListString',scenarios(:,1), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', vehicle_amount, ...
        'PromptString', 'Choose the amount of vehicles');

    if(~ok)
        error('Canceled');
    end
    
    [plot_option,ok] = listdlg(...
        'ListString',possPlots(1:end-(strat_option-1),2), ... % remove option if controller is pbnc
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', plot_option, ...
        'PromptString', 'Choose the type of visualization during simulation');
   
    if(~ok)
        error('Canceled');
    end


    
else
    vehicle_amount = vehicle_amount_p;
    strat_option = strat_option_p;
    plot_option = plot_option_p;
    scenario_option = scenario;
    priority_option = priority_assignment;
end

options.isPB = (strat_option == 2);
options.angles = scenarios{vehicle_amount,2};
options.amount = vehicle_amount;
options.visu = possPlots{plot_option,3};
options.scenario = scenarioName{scenario_option,2};
options.priority = priorityAssignment{priority_option,2};


    % ============= save choice ============
    save([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option', 'strat_option','scenario_option','priority_option');

end
