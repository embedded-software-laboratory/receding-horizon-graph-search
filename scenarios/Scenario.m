classdef Scenario
    properties
        vehicles = [];  % array of Vehicle objects
        % obstacles = {[xs;ys],...}
        obstacles = {};
        nVeh = 0;
        name = 'UnnamedScenario';
        controller_name = 'RHC';
        dt;     % RHC sample time [s]
        Hp;
        Hu;
        mpa;
        trim_set = 'trim_set_3_1';
        offset = 0.1;           % offset for collision checks
        model = [];
        r_goal = 1; % goal circle
        time_per_tick;
        tick_per_step;
    end
    
    methods
        function obj = Scenario(options)
            radius = 15;
            for the_angle=options.angles
                s = sin(the_angle);
                c = cos(the_angle);
                veh = Vehicle();
                veh.x_start = -c*radius;
                veh.y_start = -s*radius;
                veh.yaw_start = the_angle;
                veh.x_goal = c*radius;
                veh.y_goal = s*radius;
                veh.yaw_goal = the_angle;
                veh.trim_config = 1;
                veh.referenceTrajectory = [-c*radius -s*radius;c*radius s*radius]; 
                obj.vehicles = [obj.vehicles, veh];
            end
            obj.nVeh = options.amount;
            obj.name = sprintf("%i-circle", options.amount);
            obj.Hp = 6;
            obj.Hu = obj.Hp;
            
            obj.dt = 0.4;
            obj.time_per_tick = 0.01;
            obj.tick_per_step = obj.dt/obj.time_per_tick;


            
            % Make motionGraph Tupel
            obj.model = BicycleModel(2.2,2.2);
            load(obj.trim_set); % loads u_trims and trim_adjacency
            % Combine graphs
            obj.mpa = MotionPrimitiveAutomaton(obj.model, u_trims, trim_adjacency, obj.offset, obj.dt, options.amount);
        end
        
        function plot(obj, varargin)
            if nargin==1
                fig = figure;
            else
                fig = varargin{1};
            end
            figure(fig);
            veh_colors = [  0.8941    0.1020    0.1098  ;...
                            0.2157    0.4941    0.7216  ;...
                            0.3020    0.6863    0.2902  ;...
                            0.5961    0.3059    0.6392  ;...
                            1.0000    0.4980    0       ;...
                            1.0000    1.0000    0.2000  ;...
                            0.6510    0.3373    0.1569  ;...
                            0.9686    0.5059    0.7490  ];
            for iVeh = 1:numel(obj.vehicles)
                hold on
                % vehicle rectangle
                veh = obj.vehicles(iVeh);
                veh.plot(fig, veh_colors(mod(iVeh-1,size(veh_colors,1))+1,:));
                hold off
            end
            axis equal
        end
    end
    
    
end