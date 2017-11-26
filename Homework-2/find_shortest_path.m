function [ valid_path, path ] = find_shortest_path( waypoint1, waypoint2,...
    milestones_x, milestones_y, adj_set, map, resolution )
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
                collision = collision_detected(path(i-1, :), path(i, :), map, resolution);

                if (collision)
                    adj_set(path(i-1), path(i))   = 0;
                    adj_set(path(i),   path(i-1)) = 0;
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
