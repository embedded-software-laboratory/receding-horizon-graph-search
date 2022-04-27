classdef  random_priority < interface_priority
% random_priority  Instance of interface_priority used for dynamic priority
% assignment, randomly assign priority to vehicles
    
    properties (Access=private)

        
    end
    
    methods 
        function obj = random_priority(scenario)
            obj.scenario = scenario;
        end
        
        function groups = priority(obj)
            
%             groups = PB_random_groups(obj.scenario);
            groups = struct;
            nVeh = length(obj.scenario.vehicles);
            randomPrio = randperm(nVeh,nVeh); % random priority order 

            for group_idx = 1:nVeh
                groups(group_idx).members = randomPrio(group_idx);
                if group_idx == 1
                    groups(group_idx).predecessors = [];
                else
                    groups(group_idx).predecessors = [groups(group_idx-1).predecessors groups(group_idx-1).members];
                end
            end

            
        end
      
        
    end
    


end