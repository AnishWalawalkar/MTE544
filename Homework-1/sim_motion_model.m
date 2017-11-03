function [ x_true, y, x_est, x_cov ] = sim_motion_model(A, B, C, D, u, cov_dist, cov_meas, T, dt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


    rot = eye(3);
    x_prev = [0; 0; 0];
    x_true = [];
    y = [];
    decl = [0; 0; -9.7*pi/180];

    R = cov_dist;
    Q = cov_meas;
    mu_p = x_prev;
    x_est = x_prev;
    cov_p = 3.*eye(3);
    x_cov = 3.*eye(3);
    
    cov_meas_corr = [0.01 0 0;
                           0    0.01 0;
                           0    0    10*pi/180].^2;
    
    [REx, Rex] = eig(cov_dist);
    [REy, Rey] = eig(cov_meas);
    [REycorr, Reycorr] = eig(cov_meas_corr);

    for i = 2:T
        % True State Update
        Ex = REx*sqrt(Rex)*randn(3,1);
        
        x_true(:, i) = A*x_prev + rot*B*u(:, i) + Ex;
        rot = [cos(x_true(3, i)) -sin(x_true(3, i)) 0;
               sin(x_true(3, i)) cos(x_true(3, i))  0;
               0            0             1];
        % Generate GPS and MAG measurements
        
        if (mod(T, 10) == 0)
            Ey = REycorr*sqrt(Reycorr)*randn(3,1);
        else
            Ey = REy*sqrt(Rey)*randn(3,1);
        end
               
        y(:, i) = C*x_true(:, i) + D*u(:, i) + Ey + decl;
        x_prev = x_true(: , i);

        % EKF Prediction Update
        mu_p(:, i) = A*x_est(:, i-1) + rot*B*u(:, i);
        cov_p(:, :, i) = A*x_cov(:, :, i-1) + R;
        
        % EKF Measurement Update
        if (mod(T, 10) == 0)
            Q = cov_meas_corr;
        else
            Q = cov_meas;
        end
        
        K = cov_p(:, :, i)*C'*inv(C*cov_p(:, :, i)*C' + Q);
        x_est(:, i) = mu_p(:, i) + K*(y(:, i)-decl - C*mu_p(:, i));
        x_cov(:, :, i) = (eye(3) - K*C)*cov_p(:, :, i);
    end

end

