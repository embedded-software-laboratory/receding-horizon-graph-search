function options = selection(vehicle_amount_p,plot_option_p,strat_option_p)
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
        load([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option', 'strat_option');
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
    
    [strat_option,ok] = listdlg(...
        'ListString',possStrats(:,2), ...
        'ListSize', [300,300], ...
        'SelectionMode', 'single', ...
        'InitialValue', strat_option, ...
        'PromptString', 'Choose the control strategy');

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
end

options.isPB = (strat_option == 2);
options.angles = scenarios{vehicle_amount,2};
options.amount = vehicle_amount;
options.visu = possPlots{plot_option,3};


    % ============= save choice ============
    save([tempdir 'uiSelection'], 'vehicle_amount', 'plot_option', 'strat_option');

end
