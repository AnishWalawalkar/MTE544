function [ valid_path, path ] = prm( map, waypoint1, waypoint2 )
    % define constants
    num_milestones = 300;
    map_resolution = 0.1;
    max_values = size(map) * map_resolution;
    min_values = [0 0] * map_resolution;
    diff_values = max_values - min_values;
    
    while true
        % generate milestones
        milestones = [];
        count = 1;

        while count < num_milestones
            x = diff_values(1)*rand(1);
            y = diff_values(2)*rand(1);

            if ~map(ceil(x/map_resolution), ceil(y/map_resolution))
               milestones(count, :) = [x y]; 
               count = count + 1;
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

        [valid_path, path] = find_shortest_path(waypoint1, waypoint2,...
            milestones_x, milestones_y, adjacency_set, map, map_resolution);
        
        if (valid_path); break; end;
    end
    
    % Plotting
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot(waypoint1(1)/map_resolution, waypoint1(2)/map_resolution, 'ro', 'MarkerSize',10, 'LineWidth', 3);
    plot(waypoint2(1)/map_resolution, waypoint2(2)/map_resolution, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
    scatter(milestones_x/map_resolution, milestones_y/map_resolution, 'x');
    axis equal
    
end

