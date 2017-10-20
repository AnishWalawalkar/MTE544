clc;
clear all;
close all;

%% define variables
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

e = [0.01; 0.01; 0.1]; % verify variance / std




%% drive in cirle
rot = eye(3);
x_prev = [0; 0; 0];
x = [];

for i = 1:150
    dist = randn(3,1) .* e;
    x(:, i) = x_prev + rot*B*u*dt + dist;
    rot = [cos(x(3, i)) -sin(x(3, i)) 0;
        sin(x(3, i)) cos(x(3, i)) 0;
        0 0 1];
    x_prev = x(: , i);
    
end

figure;
plot(x(1, :), x(2, :));

%% drive in straight line

v = 1;
theta = pi/4;

u = inv(B) * [v*cos(theta); v*sin(theta); 0];

rot = eye(3);
x_prev = [0; 0; 0];
x = [];

for i = 1:150
    dist = randn(3,1) .* e;
    x(:, i) = x_prev + rot*B*u*dt + dist;
    rot = [cos(x(3, i)) -sin(x(3, i)) 0;
        sin(x(3, i)) cos(x(3, i)) 0;
        0 0 1];
    x_prev = x(: , i);
    
end

figure;
plot(x(1, :), x(2, :));


%% drive in spiral

rot = eye(3);
x_prev = [0; 0; 0];
x = [];
u = [1; 0; 2]

for i = 1:150
    dist = randn(3,1) .* e;
    x(:, i) = x_prev + rot*B*u*dt;
    rot = [cos(x(3, i)) -sin(x(3, i)) 0;
        sin(x(3, i)) cos(x(3, i)) 0;
        0 0 1];
    u(3) = 1/((i*dt)^2+0.5)
    x_prev = x(: , i);
    
end

figure;
plot(x(1, :), x(2, :));


