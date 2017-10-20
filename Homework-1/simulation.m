clc;
clear all;
close all;

l = 0.3;
r = 0.25;


w1 = -1.5;
w2 = 2.0;
w3 = 1.0;
u = [w1; w2; w3];

dt = 0.1;

A = eye(3);

B = r * [0 -sqrt(3)/2 sqrt(3)/2; 
    1 -1/2 -1/2;
    1/(3*l) 1/(3*l) 1/(3*l)];

e = [0.01; 0.01; 0.1];

rot = eye(3);
x_prev = [0; 0; 0];

x = zeros(3, 150);
%%
for i = 1:60 
    dist = randn(3,1) .* e;
    x(:, i) = rot*x_prev + rot*B*u*dt + dist;
    rot = [cos(x(3, i)) -sin(x(3, i)) 0;
        sin(x(3, i)) cos(x(3, i)) 0;
        0 0 1];
    x_prev = x(: , i);
    
end


plot(x(1, :), x(2, :));
