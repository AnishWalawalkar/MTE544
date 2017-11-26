function [ distance ] = distance_nodes( n1, n2, nodes_x, nodes_y )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    distx = nodes_x(n2) - nodes_x(n1);
    disty = nodes_y(n2) - nodes_y(n1);
    distance = sqrt(distx^2 + disty^2);

end

