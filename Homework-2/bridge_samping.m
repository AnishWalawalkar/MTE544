function [ milestones ] = bridge_samping( map )
    % define constants
    num_milestones = 400;
    map_resolution = 0.1;
    max_values = size(map) * map_resolution;
    min_values = [1 1];
    diff_values = max_values - min_values;
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
end

