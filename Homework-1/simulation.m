clc;
clear all;
close all;

%% define variables
l = 0.3;
r = 0.25;

T = 150;
dt = 0.1;

A = eye(3);

B = r*dt*[0       -sqrt(3)/2 sqrt(3)/2; 
          1       -1/2       -1/2;
          1/(3*l) 1/(3*l)    1/(3*l)];
C = eye(3);
D = zeros(3);
B_inv = inv(B);
cov_dist = [0.01 0    0;
            0    0.01 0;
            0    0    0.1].^2; % verify variance / std
cov_meas = [0.5 0   0;
            0   0.5 0;
            0   0   10*pi/180].^2; % verify variance / std





%% drive with wheel speeds
w1 = -1.5;
w2 = 2.0;
w3 = 1.0;
u = [w1; w2; w3];
u = repmat(u, 1, T/dt);

[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, T);

figure(1);
plot(x_true(1, :), x_true(2, :));

hold on
scatter(y(1, :), y(2, :), '.');
plot(x_est(1, :), x_est(2, :), '-.');
for i = 1:5:150
    error_ellipse(x_cov(1:2, 1:2, i), x_est(1:2, i), 'style', 'm');
end
legend('True State', 'GPS Measurements', 'Estimated State', 'Error Ellipses');
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('EKF Implementation for Three Omni-Wheeled Robot')

%% drive in cirle

v = 1;


for i = 1:T/dt
    u(:, i) = B\[v*cos(i*dt); v*sin(i*dt); 0];
end

[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, T);

figure(2);
plot(x_true(1, :), x_true(2, :));

%% drive in straight line

v = 1;
theta = pi/4;

u = B\[v*cos(theta); v*sin(theta); 0];
u = repmat(u, 1, T/dt);

[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, T);

figure(3);
plot(x_true(1, :), x_true(2, :));


%% drive in spiral

for i = 1:T/dt
    u(:, i) = B\[cos(i*dt) - i*dt*sin(i*dt); sin(i*dt)+i*dt*cos(i*dt); 0];
end

[x_true, y, x_est, x_cov] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, T);

figure(4);
plot(x_true(1, :), x_true(2, :));












