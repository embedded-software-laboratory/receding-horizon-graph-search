classdef Simulation < RunningEnv
% CPMLAB    Instance of running environment for usage as simulation in matlab.
    
    properties (Access=private)
        doOnlinePlot
        doExploration
        fig
        resolution
        paused
        abort
    end
    
    methods
        function obj = Simulation(scenario, options)
            obj.doOnlinePlot = options.visu(1);
            obj.doExploration = options.visu(2);
            obj.scenario = scenario;
            obj.paused = false;
            obj.abort = false;
            obj.cur_node = node(0, [obj.scenario.vehicles(:).trim_config], [obj.scenario.vehicles(:).x_start]', [obj.scenario.vehicles(:).y_start]', [obj.scenario.vehicles(:).yaw_start]', zeros(obj.scenario.nVeh,1), zeros(obj.scenario.nVeh,1));
        end
        
        function keyPressCallback(obj, ~, eventdata)
            if strcmp(eventdata.Key, 'escape')
                obj.abort = true;
            elseif strcmp(eventdata.Key, 'space')
                obj.paused = ~obj.paused;
            end
        end
        
        function setup(obj)
            if obj.doOnlinePlot
                obj.resolution = [1920 1080];
                obj.fig = figure(...
                    'Visible','On'...
                    ,'Color',[1 1 1]...
                    ,'units','pixel'...
                    ,'OuterPosition',[100 100 obj.resolution(1) obj.resolution(2)]...
                );
                set(gcf,'WindowKeyPressFcn',@obj.keyPressCallback);
                hold on
            end
        end
        
        function [ x0, trim_indices ] = measure(obj)
            % take last planned state as new actual state
            speeds = zeros(obj.scenario.nVeh,1);
            for iVeh=1:obj.scenario.nVeh
                speeds(iVeh) = obj.scenario.mpa.trims(obj.cur_node(iVeh,NodeInfo.trim)).speed;
            end
            x0 = [obj.cur_node(:,NodeInfo.x), obj.cur_node(:,NodeInfo.y), obj.cur_node(:,NodeInfo.yaw), speeds];
            trim_indices = obj.cur_node(:,NodeInfo.trim);
        end
        
        function apply(obj, ~, ~, info, result, k)
            % init struct for exploration plot
            if obj.doExploration
                exploration_struct.doExploration = true;
                exploration_struct.info = info;
            else
                exploration_struct = [];
            end
            % get current iteration index
            obj.k = k;
            if obj.doOnlinePlot
                % wait to simulate realtime plotting
                pause(obj.scenario.dt-result.step_time(obj.k))

                % visualize time step
                plotOnline(result,obj.k,1,exploration_struct);
            end
            
            obj.cur_node = info.next_node;
        end
        
        function got_stop = is_stop(obj)
            got_stop = false;
            % idle while paused, and check if we should stop early
            while obj.paused
                pause(0.1);
                if obj.abort
                    disp('Aborted.');
                    got_stop = true;
                    break;
                end
            end
            if obj.abort
                disp('Aborted.');
                got_stop = true;
            end
            if  obj.k >= obj.scenario.k_end
                got_stop = true;
            end
        end
        
        function end_run(obj)
            disp('End')
            if obj.doOnlinePlot
                close(obj.fig)
            end
        end
    end
end

