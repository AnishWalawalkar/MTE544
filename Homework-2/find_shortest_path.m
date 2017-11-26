function [ valid_path, path ] = find_shortest_path( waypoint1, waypoint2, milestones_x, milestones_y, adj_set )
    valid_path = false;
    path = [];
    while ~valid_path
        [found, path] = a_star(waypoint1, waypoint2, milestones_x, milestones_y, adj_set);
        if (~found); break; 
        else
            collision = false;
            for i=2:length(path)
                prev = path(i-1, :);
                curr = path(i, :);
                collision = collision_detected(path(i-1, :), path(i, :));
                
                if (collision)
                    idx0 = find(prev(1));
                    idx1 = find(curr(1));
                    adj_set(idx0, idx1) = 0;
                    adj_set(idx1, idx0) = 0;
                end
            end
            
            if(collision)
                continue;
            else
                valid_path = true;
            end
        end
    end
end

