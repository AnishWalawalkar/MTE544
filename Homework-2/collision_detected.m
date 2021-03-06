function [ detected ] = collision_detected( point1, point2, map, resolution )
    point1 = round(point1/resolution);
    point2 = round(point2/resolution);
    
%     [line_x, line_y] = generate_line(point1(1), point2(1), ...
%         point1(2), point2(2));
    [line_x, line_y] = bresenham(point1(1), point1(2)...
            , point2(1), point2(2));
    
    for i = 1:length(line_x)
        curr_x = line_x(i);
        curr_y = line_y(i);
        
        if (map(curr_x+1, curr_y+1) == 1)
            detected = true;
            return;
        end
    end
    
    detected = false;
end

