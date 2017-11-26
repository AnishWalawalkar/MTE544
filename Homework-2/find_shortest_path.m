function [ valid_path, path ] = find_shortest_path( first, last, milestones_x, milestones_y, adj_set, map, resolution )
    valid_path = false;
    path = [];
    good_paths = zeros(size(adj_set));
    while ~valid_path
        [found, path] = a_star(first, last, milestones_x, milestones_y, adj_set);
        if (~found); break; 
        else
            valid_path = true;
            for i=2:length(path)
                prev_point = [milestones_x(path(i-1)), milestones_y(path(i-1))];
                curr_point = [milestones_x(path(i)),   milestones_y(path(i))];
                collision = false;
                if (~good_paths(path(i-1), path(i)))
                    collision = collision_detected(prev_point, curr_point, map, resolution);
                end
                
                if (collision)
                    valid_path = false;
                    adj_set(path(i-1), path(i))   = 0;
                    adj_set(path(i),   path(i-1)) = 0;
                else
                    good_paths(path(i-1), path(i))   = 1;
                    good_paths(path(i),   path(i-1)) = 1;
                end
            end
        end
    end
end