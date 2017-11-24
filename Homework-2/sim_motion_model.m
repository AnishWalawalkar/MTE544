function [ x ] = sim_motion_model(u, cov_dist, T, dt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    L = 0.3;
    A = eye(3);
    x = zeros(3,1);

    [REx, Rex] = eig(cov_dist);

    for k = 1:length(T)
        Ex = REx*sqrt(Rex)*randn(3,1);

        Bu = [cos(x(3,k));
              sin(x(3,k));
              tan(u(2,k))/L;]*u(1,k)*dt;

        x(:, k+1) = A*x(:, k) + Bu + Ex;
    end

end

