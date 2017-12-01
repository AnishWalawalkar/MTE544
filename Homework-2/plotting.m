map = read_map('IGVCmap.jpg');
milestones_random = random_sampling(map);
milestones_bridge = bridge_samping(map);

figure(1);
subplot(1,2,1);
hold on;
colormap('gray');
imagesc(1-map');
scatter(milestones_random(:,1)/map_resolution, milestones_random(:, 2)/map_resolution, 'x');
l = legend('Random Samples');
l.FontSize = 18;
hold off;

subplot(1,2,2);
hold on;
colormap('gray');
imagesc(1-map');
scatter(milestones_bridge(:,1)/map_resolution, milestones_bridge(:, 2)/map_resolution, 'x');
l = legend('Bridge Samples');
l.FontSize = 18;
hold off;