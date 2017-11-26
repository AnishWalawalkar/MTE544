function [ map ] = read_map( map_name )
    I = imread(map_name);
    map = im2bw(I, 0.7); % Convert to 0-1 image
    map = 1-flipud(map)'; % Convert to 0 free, 1 occupied and flip.
end

