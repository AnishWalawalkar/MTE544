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
            0    0    0.1*pi/180].^2;
cov_meas = [0.5 0   0;
            0   0.5 0;
            0   0   10*pi/180].^2;

cov_meas_corr = [0.01 0    0;
                 0    0.01 0;
                 0    0    10*pi/180].^2;



%% Drive with wheel speeds EKF
w1 = -1.5;
w2 = 2.0;
w3 = 1.0;
u = [w1; w2; w3];
u = repmat(u, 1, T/dt);

[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, cov_meas, T);

figure(8)
hold on
plot(x_true(1, :), x_true(2, :));
scatter(y(1, :), y(2, :), '.');
plot(x_est(1, :), x_est(2, :), 'g-.');
for i = 1:3:150
    error_ellipse(x_cov(1:2, 1:2, i), x_est(1:2, i), 'style', 'm');
end
legend('True State', 'GPS Measurements', 'Estimated State', 'Error Ellipses');
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('EKF Implementation for Three Omni-Wheeled Robot')
hold off

figure(1);
subplot(2, 2, 1)
% X Y Plot of the True State, Measurements and Error
hold on
plot(x_true(1, :), x_true(2, :));
scatter(y(1, :), y(2, :), '.');
plot(x_est(1, :), x_est(2, :), 'g-.');
for i = 1:3:150
    error_ellipse(x_cov(1:2, 1:2, i), x_est(1:2, i), 'style', 'm');
end
legend('True State', 'GPS Measurements', 'Estimated State', 'Error Ellipses');
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('EKF Implementation for Three Omni-Wheeled Robot')
hold off


subplot(2, 2, 2)
time = dt:dt:15;
plot(time, x_true(1, :), time, x_est(1, :));
legend('True State', 'Estimated State')
ylabel('North [m]')
xlabel('Time [s]')
title('EKF Estimation vs True State in X')

subplot(2, 2, 3)
time = dt:dt:15;
plot(time, x_true(2, :), time, x_est(2, :));
legend('True State', 'Estimated State')
ylabel('West [m]')
xlabel('Time [s]')
title('EKF Estimation vs True State in Y')

subplot(2, 2, 4)
time = dt:dt:15;
plot(time, x_true(3, :), time, x_est(3, :));
legend('True State', 'Estimated State')
ylabel('Yaw [rad]')
xlabel('Time [s]')
title('EKF Estimation vs True State in Yaw')
%% Drive with wheel speeds Multirate EKF
[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, cov_meas_corr, T);

figure(7)
plot(x_true(1, :), x_true(2, :));

hold on
scatter(y(1, :), y(2, :), '.');
plot(x_est(1, :), x_est(2, :), 'g-.');
for i = 1:1:150
    error_ellipse(x_cov(1:2, 1:2, i), x_est(1:2, i), 'style', 'm');
end
legend('True State', 'GPS Measurements', 'Estimated State', 'Error Ellipses');
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('Multi Rate EKF Implementation for Three Omni-Wheeled Robot')
hold off
figure(2);
subplot(2, 2, 1)
plot(x_true(1, :), x_true(2, :));

hold on
scatter(y(1, :), y(2, :), '.');
plot(x_est(1, :), x_est(2, :), 'g-.');
for i = 1:1:150
    error_ellipse(x_cov(1:2, 1:2, i), x_est(1:2, i), 'style', 'm');
end
legend('True State', 'GPS Measurements', 'Estimated State', 'Error Ellipses');
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('Multi Rate EKF Implementation for Three Omni-Wheeled Robot')
hold off

subplot(2, 2, 2)
time = dt:dt:15;
plot(time, x_true(1, :), time, x_est(1, :));
legend('True State', 'Estimated State')
ylabel('North [m]')
xlabel('Time [s]')
title('EKF Estimation vs True State in X')

subplot(2, 2, 3)
time = dt:dt:15;
plot(time, x_true(2, :), time, x_est(2, :));
legend('True State', 'Estimated State')
ylabel('West [m]')
xlabel('Time [s]')
title('EKF Estimation vs True State in Y')

subplot(2, 2, 4)
time = dt:dt:15;
plot(time, x_true(3, :), time, x_est(3, :));
legend('True State', 'Estimated State')
ylabel('Yaw [rad]')
xlabel('Time [s]')
title('EKF Estimation vs True State in Yaw')


%% Drive with wheel speeds simulation
w1 = -1.5;
w2 = 2.0;
w3 = 1.0;
u = [w1; w2; w3];
u = repmat(u, 1, T/dt);


[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, zeros(3,3), cov_meas, cov_meas, T);

figure(3);
subplot(1,2,1);
plot(x_true(1,:), x_true(2, :));
camroll(90);
xlabel('North [m]');
ylabel('West [m]');
title('True robot state simulation: given inputs without noise')

[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, cov_meas, T);

subplot(1,2,2);
plot(x_true(1,:), x_true(2, :));
camroll(90);
xlabel('North [m]');
ylabel('West [m]');
title('True robot state simulation: given inputs with noise')


%% drive in 2m cirle

v = 1*dt;

for i = 1:T
    u(:, i) = B\[v*cos(i*dt); v*sin(i*dt); 0];
end


cov_dist = zeros(3,3);
cov_meas = zeros(3, 3);

[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, zeros(3,3), cov_meas, cov_meas, T);


figure(4);
subplot(1,2,1);
plot(x_true(1,:), x_true(2, :));
camroll(90);
xlabel('North [m]')
ylabel('West [m]')
title('True robot state simulation: driving in 2m circle without noise')


[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, cov_meas, T);

subplot(1,2,2);
plot(x_true(1,:), x_true(2, :));
camroll(90);
xlabel('North [m]')
ylabel('West [m]')
title('True robot state simulation: driving in 2m circle with noise')
%% drive in straight line

v = 1*dt;
theta = pi/4;

u = B\[v*cos(theta); v*sin(theta); 0];
u = repmat(u, 1, T/dt);


[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, zeros(3,3), cov_meas, cov_meas, T);


figure(5);
subplot(1,2,1);
plot(x_true(1,:), x_true(2, :));
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('True robot state simulation: driving in straight line without noise')


[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, cov_meas, T);

subplot(1,2,2);
plot(x_true(1,:), x_true(2, :));
camroll(90);
xlabel('North [m]')
ylabel('West [m]')
title('True robot state simulation: driving in straight line with noise')

%% drive in spiral

v = 1*dt;

for i = 1:T
    u(:, i) = B\[v*(cos(i*dt) - i*dt*sin(i*dt)); v*(sin(i*dt)+i*dt*cos(i*dt)); 0];
end


[x_true, y, x_est, x_cov] = sim_motion_model(A, B, C, D, u, zeros(3,3), cov_meas, cov_meas, T);


figure(6);
subplot(1,2,1);
plot(x_true(1,:), x_true(2, :));
camroll(90)
xlabel('North [m]')
ylabel('West [m]')
title('True robot state simulation: driving in a spiral without noise')


[ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, cov_meas, T);

subplot(1,2,2);
plot(x_true(1,:), x_true(2, :));
camroll(90);
xlabel('North [m]')
ylabel('West [m]')
title('True robot state simulation: driving in a spiral with noise')
