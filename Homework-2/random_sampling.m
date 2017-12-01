function [ milestones ] = random_sampling( map )
    num_milestones = 400;
    map_resolution = 0.1;
    max_values = size(map) * map_resolution;
    min_values = [1 1];
    diff_values = max_values - min_values;
    
    
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

end

