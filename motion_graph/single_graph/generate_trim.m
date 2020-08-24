% genrates trim struct
function trim = generate_trim(model, trim_input)
    
    % BicycleModel
    if model.nu == 2
        
        trim = struct('steering',0,'velocity',0);
        trim.steering = trim_input(1);
        trim.velocity = trim_input(2); 
        
    % BicycleModelConstSpeed
    elseif model.nu == 1
        
        trim = struct('steering',0);
        trim.steering = trim_input(1);
        
    end

end