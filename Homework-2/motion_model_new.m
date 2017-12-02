close all;

dt = 0.1;
L = 0.3;

v = 3;
cov_dist = [0.02 0 0;
            0 0.02 0;
            0 0 pi/180;].^2;
u = [];
T = 1:dt:20;


x = [0; 0; 0];

[REx, Rex] = eig(cov_dist);



for k = 2:length(T)
	% Motion Model
    Ex = REx*sqrt(Rex)*randn(3,1);
    
    x(1, k) = x(1, k-1) + v*cos(x(3,k-1))*dt + Ex(1); 
    x(2, k) = x(2, k-1) + v*sin(x(3,k-1))*dt + Ex(2); 
    
    ang = (10-(k*dt))*pi/180;
    if ang > pi/6
        ang = pi/6 - 2*pi;
    elseif ang < -pi/6
        ang = -pi/6 + 2*pi;
    end
    x(3, k) = x(3, k-1) + v*dt*tan(ang)/L + Ex(3); 
end

plot(x(1,:), x(2,:), 'bx');
title('Ackermann Bicycle Driving for 20 [sec] with Changing Steering Angle', 'FontSize', 16);
xlabel('x [m]', 'FontSize', 16);
ylabel('y [m]', 'FontSize', 16);
axis equal;



