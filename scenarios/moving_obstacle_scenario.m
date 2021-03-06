function scenario = moving_obstacle_scenario(is_start_end)
% MOVING_OBSTACLE_SCENARIO     Constructor for moving obstacle scenario
    arguments
        is_start_end (1,1) logical = 0;
    end
    scenario = Scenario();
    scenario.trim_set = 3;
    veh = Vehicle();
    radius = 3;
    center_x = 2.25;
    center_y = 0;
    veh.x_start = -radius + center_x - 2;
    veh.y_start = 0 + center_y;
    veh.yaw_start = 0;
    veh.x_goal = radius + center_x + 100;
    veh.y_goal = 0 + center_y;
    veh.yaw_goal = 0;
    veh.trim_config = 1;
    veh.referenceTrajectory = [veh.x_start veh.y_start;veh.x_goal veh.y_goal];
    scenario.vehicles = veh;
    scenario.nVeh = 1;
    scenario.T_end = 10;
    if ~is_start_end
        scenario.Hp = 5;
        scenario.name = sprintf('moving_obstacles');
    else
        scenario.Hp = scenario.T_end / scenario.dt;
        scenario.name = sprintf('moving_obstacles_start_end');
    end
    
    scenario.model = BicycleModel(veh.Lf,veh.Lr);
    
    scenario.plot_limits = [-4,5;-1.5,1.5];
    
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

    x1_obs = veh.x_start + 2.2;
    x2_obs = x1_obs + 1.5;
    
    y1_obs = 4.1;
    y2_obs = -4;
    
    dist_obs = 1.7;
    speed_obs = 0.75;
    
    dist_dt = speed_obs*scenario.dt; % distance traveled per timestep
    width_obs = 0.5;
    length_obs = 0.5;
    length_coll_area = length_obs+dist_dt;
    
    scenario.dynamic_obstacle_shape = [width_obs;length_obs];
    
    
    for iObstacle = 1:20
        for iTimestep = 1:80
            scenario.dynamic_obstacle_area{iObstacle,iTimestep} = transformedRectangle(...
                x1_obs ...
                ,center_y+y1_obs + iTimestep*dist_dt - iObstacle*dist_obs ...
                ,pi/2 ...
                ,length_coll_area ...
                ,width_obs ...
            );
            scenario.dynamic_obstacle_fullres{iObstacle,iTimestep} = [  ones(41,1)*x1_obs, ...
                                                        linspace(   center_y+y1_obs + iTimestep*dist_dt - iObstacle*dist_obs - dist_dt/2, ...
                                                                    center_y+y1_obs + (iTimestep+1)*dist_dt - iObstacle*dist_obs - dist_dt/2, 41)'];
                                                                
           scenario.dynamic_obstacle_area{iObstacle+20,iTimestep} = transformedRectangle(...
                x2_obs ...
                ,center_y+y2_obs - iTimestep*dist_dt + iObstacle*dist_obs ...
                ,pi/2 ...
                ,length_coll_area ...
                ,width_obs ...
            );
            scenario.dynamic_obstacle_fullres{iObstacle+20,iTimestep} = [  ones(41,1)*x2_obs, ...
                                                        linspace(   center_y+y2_obs - iTimestep*dist_dt + iObstacle*dist_obs + dist_dt/2, ...
                                                                    center_y+y2_obs - (iTimestep+1)*dist_dt + iObstacle*dist_obs + dist_dt/2, 41)'];
        end
    end
    
end