function [ valid_path, path, milestones_x, milestones_y ] = prm( map, waypoint1, waypoint2 )
    % define constants
    num_milestones = 100;
    map_resolution = 0.1;
    max_values = size(map) * map_resolution;
    min_values = [1 1];
    diff_values = max_values - min_values;
    loopcount = 0;
    while true
        % generate milestones
        loopcount = loopcount + 1
        milestones = [];
        count = 1;

        while count < num_milestones
            x1 = diff_values(1)*rand(1);
            y1 = diff_values(2)*rand(1);
            x2 = 10*randn + x1;
            y2 = 10*randn + y1;
            xmid = (x1+x2)/2;
            ymid = (y1+y2)/2;
            if (round(x2/map_resolution) > 0 &&...
                round(y2/map_resolution) > 0 &&...
                round(x2/map_resolution) < size(map, 1) &&...
                round(y2/map_resolution) < size(map, 2) &&...
                round(x1/map_resolution) > 0 &&...
                round(y1/map_resolution) > 0 &&...
                round(x1/map_resolution) < size(map, 1) &&...
                round(y1/map_resolution) < size(map, 2))
                if (map(round(x1/map_resolution), round(y1/map_resolution)) &&...
                    map(round(x2/map_resolution), round(y2/map_resolution)) &&...
                    ~map(round(xmid/map_resolution), round(ymid/map_resolution)))
                   milestones(count, :) = [xmid ymid]; 
                   count = count + 1;
                end
            end
        end

        milestones(count+1, :) = waypoint1;
        milestones(count+2, :) = waypoint2;

        milestones_x = milestones(:, 1);
        milestones_y = milestones(:, 2);


        % generate adjacency set
        adjacency_set = eye(num_milestones);
        milestones_connection_raduis = 45;
        dist = @(point1, point2) sqrt((point1(1)-point2(1))^2 + (point1(2)-point2(2))^2);

        for i = 1:length(milestones)
            for j = 1:length(milestones)
                if i == j
                    continue
                end

                if (dist(milestones(i,:), milestones(j,:)) < milestones_connection_raduis)
                    adjacency_set(i, j) = 1;
                    adjacency_set(j, i) = 1;
                end
            end
        end

        % find shortest path and validate

        [valid_path, path] = find_shortest_path(count+1, count+2,...
            milestones_x, milestones_y, adjacency_set, map, map_resolution);
        
        if (valid_path); break; end;
    end
    
    
end

