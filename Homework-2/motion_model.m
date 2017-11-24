r = 1;
K_STEER = 1;
dt = 0.1;
L = 0.3;
A = eye(3);
cov_dist = [0.02 0 0;
            0 0.02 0;
            0 0 pi/180;].^2;

u = [];
T = 1:dt:20;

for k = 1:length(T)
   u(:,k) = [3;
             (10-T(k))*pi/180];
end


x = zeros(3,1);

[REx, Rex] = eig(cov_dist);

points_x = [0, 20, 20, 0, 0];
points_y = [0, 0,   5, 5, -1];

for k = 1:length(T)
    % Controller
    path_ang = atan2(points_y(2) - points_y(1),...
                     points_x(2) - points_x(1));
    curr_ang = atan2(x(2,k) - points_y(1),...
                     x(1,k) - points_x(1));
    diff_ang = path_ang - curr_ang;
    
    distance = sqrt((x(2,k) - points_y(1))^2 + (x(1,k) - points_x(1))^2);

    err_d = distance * sin(diff_ang);
    diff_dist = distance*cos(diff_ang) - sqrt((points_y(2) - points_y(1))^2 + (points_x(2) - points_x(1))^2);

    carrot_x = (distance*cos(diff_ang) + r)*cos(path_ang) + points_x(1);
    carrot_y = (distance*cos(diff_ang) + r)*sin(path_ang) + points_y(1);
    if (diff_dist + r > 0)
        if (size(points_x) > 3)
            path2_ang = atan2(points_y(3) - points_y(2),...
                              points_x(3) - points_x(2));
            carrot_x = (diff_dist)*cos(path2_ang) + points_x(2);
            carrot_y = (diff_dist)*sin(path2_ang) + points_y(2);
        else
            carrot_x = points_x(2);
            carrot_y = points_y(2);
        end
    end
    
    if (abs(diff_dist) < r)
        points_x = points_x(2:end);
        points_y = points_y(2:end);
        if (size(points_x) == 1)
            break;
        end
    end
    
    carr_ang =  atan2(carrot_y - x(2,k),...
                      carrot_x - x(1,k));
    err_h = carr_ang - x(3,k);
    u(2,k) = K_STEER * err_h;

	% Motion Model
    Ex = REx*sqrt(Rex)*randn(3,1);
    Bu = [cos(x(3,k));
          sin(x(3,k));
          tan(u(2,k))/L;]*u(1,k)*dt;

    x(:, k+1) = A*x(:, k) + Bu + Ex;
end

plot(x(1,:), x(2,:))
