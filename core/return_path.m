function search_paths = return_path(search_tree)
%RETURN_PATH returns the path to the closest node
    
    nVeh = length(search_tree.Node{1, 1}.values);
    
    min_value = Inf(1, nVeh);
    min_id = zeros(1, nVeh);
    
    % Find the node closest to the target for every vehicle
    for i = 1:nVeh
        
        for j = 1:length(search_tree.Node)
            
            if search_tree.Node{j, 1}.values(i) < min_value(i)
                min_value(i) = search_tree.Node{j, 1}.values(i);
                min_id(i) = j;
            end
            
        end
        
    end
    
    search_paths = [];
    
    max_length_path = 0;
    for i = 1:nVeh
        path = findpath(search_tree, 1, min_id(i));
        cur_length_path = length(path); 
        max_length_path = max(max_length_path, cur_length_path);
    end
    
    for i = 1:nVeh
        
    path = findpath(search_tree, 1, min_id(i));
    search_path = [];  
    
        for j = 1:max_length_path
            
            cur_length_path = length(path); 
            k = min(j, cur_length_path);
            
            x = search_tree.Node{path(k), 1}.xs(i);
            y = search_tree.Node{path(k), 1}.ys(i);
            yaw = search_tree.Node{path(k), 1}.yaws(i);

            % If there is a succesor in path add its trim else default to 0
            if (k + 1) <= length(path)
                next_trim = search_tree.Node{path(k + 1), 1}.trims(i);
            else
                next_trim = 0;
            end

            search_path = [search_path; x y yaw next_trim];

        end
        
        search_paths(:,:,i) = search_path;
        
    end

end


