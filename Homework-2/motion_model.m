% map = read_map('IGVCmap.jpg');
% waypoint1 = [40 5];
% waypoint2 = [50 10];
% map_resolution = 0.1;
% 
% [ valid, path, milestones_x, milestones_y ] = prm(map, waypoint1, waypoint2);

% Plotting
fig = 1;
figure(fig); clf; hold on;
colormap('gray');
imagesc(1-map');
plot(waypoint1(1)/map_resolution, waypoint1(2)/map_resolution, 'ro', 'MarkerSize',10, 'LineWidth', 3);
plot(waypoint2(1)/map_resolution, waypoint2(2)/map_resolution, 'gx', 'MarkerSize',10, 'LineWidth', 3 );
scatter(milestones_x/map_resolution, milestones_y/map_resolution, 'x');
plot(milestones_x(path)/map_resolution, milestones_y(path)/map_resolution, 'g');
axis equal




r = 1;
K_STEER = 1;
K_CT = 0;
dt = 0.1;
L = 0.3;
A = eye(3);
cov_dist = [0.02 0 0;
            0 0.02 0;
            0 0 pi/180;].^2;
u = [];
T = 1:dt:200;

for k = 1:length(T)
   u(:,k) = [3;
             (10-T(k))*pi/180];
end


x = [40; 5; pi];

[REx, Rex] = eig(cov_dist);

points_x = milestones_x(path);
points_y = milestones_y(path);

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
    if err_h > pi
        err_h = err_h - 2*pi;
    elseif err_h < -pi
        err_h = err_h + 2*pi;
    end
    u(2,k) = K_STEER * err_h + K_CT * err_d;
    u(2,k) = max(u(2,k), -30*pi/180);
    u(2,k) = min(u(2,k),  30*pi/180);

	% Motion Model
    Ex = REx*sqrt(Rex)*randn(3,1);
    Bu = [cos(x(3,k));
          sin(x(3,k));
          tan(u(2,k))/L;]*u(1,k)*dt;

    x(:, k+1) = A*x(:, k) + Bu + Ex;
    if (x(3, k+1) > pi)
        x(3, k+1) = x(3, k+1) - 2*pi;
    elseif (x(3, k+1) < -pi)
        x(3, k+1) = x(3, k+1) + 2*pi;
    end
    if ~mod(k, 10)
        drawbox(x(1,k)/map_resolution, x(2,k)/map_resolution, x(3,k), 5, fig);
    end
%     drawbox(carrot_x/map_resolution, carrot_y/map_resolution, 0, 1, fig);

end

plot(x(1,:)/map_resolution, x(2,:)/map_resolution)



