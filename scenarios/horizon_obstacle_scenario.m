function scenario = horizon_obstacle_scenario(N, d_obstacle)
% HORIZON_OBSTACLE_SCENARIO     Constructor for horizon obstacle scenario

    scenario = Scenario();
    scenario.trim_set = 3;
    veh = Vehicle();
    radius = 2;
    center_x = 2.25;
    center_y = 2;
    veh.x_start = -radius + center_x;
    veh.y_start = 0 + center_y;
    veh.yaw_start = 0;
    veh.x_goal = radius + center_x;
    veh.y_goal = 0 + center_y;
    veh.yaw_goal = 0;
    veh.trim_config = 1;
    veh.referenceTrajectory = [veh.x_start veh.y_start;veh.x_goal veh.y_goal];
    scenario.vehicles = veh;
    scenario.nVeh = 1;
    scenario.Hp = N;
    scenario.T_end = 6;
    
    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    
    recursive_feasibility = true;
    scenario.mpa = MotionPrimitiveAutomaton(...
        scenario.model...
        , scenario.trim_set...
        , scenario.offset...
        , scenario.dt...
        , scenario.nVeh...
        , scenario.Hp...
        , scenario.tick_per_step...
        , recursive_feasibility...
    );
    
    scenario.name = sprintf( ...
        'N_%s_d_%s', ...
        num2str(N, '%02i'),...
        num2str(d_obstacle, '%2.1f') ...
    );
    scenario.name = fullfile('horizon_obstacle', scenario.name);
    
    scenario.plot_limits = [-0.5,5;1.5,2.5];
    
    x_obs_l = veh.x_start + d_obstacle;
    scenario.obstacles{1} = transformedRectangle(...
        x_obs_l...
        ,center_y...
        ,pi/2 ...
        ,0.25 ...
        ,0.125 ...
    );
end
