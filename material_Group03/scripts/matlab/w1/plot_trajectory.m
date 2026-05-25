% TITLE: plot_trajectory.m
% IMPLEMENTED BY: Pol Rius (423483)
% DESCRIPTION: plots the trajectory of the robot, as well as any crashes.

clear all; clear figure; close all;

% Load ground truth data
% Columns: time, x, y, heading, vel_x, vel_y, vel_heading
gt = readmatrix("../../../controllers/supervisor/data/ground_truth.csv");
xpos = gt(:,2);
ypos = gt(:,3);
xytimes = gt(:,1);

% Load collision data
% Columns: time, collision_count
collisions = readmatrix("../../../controllers/supervisor/data/collisions.csv");
col_times = collisions(:, 1);
col_count = collisions(:, 2);

xcollisions = [];
ycollisions = [];

% Map collision time to collision place
for i=1:length(col_times)
    if isnan(col_times(i))
        break
    end
    I = find(abs(xytimes-col_times(i)) < 0.001);
    for j=1:col_count(i)
        xcollisions(length(xcollisions) + 1) = xpos(I);
        ycollisions(length(xcollisions) + 1) = ypos(I);
    end
end

xlims = [-1.3, 7];
ylims = [-1.5 6.75];

% Find first time the final position is reached (end time)
end_time_I = find(abs(xpos-xpos(end)) < 0.001 );
end_time_I = end_time_I(1);
end_time = xytimes(end_time_I);

figure(1);
t_travel = end_time-min(xytimes); %%%%%%%%%% Improve
n_collisions = length(xcollisions);
titlestr = sprintf("Trajectory of the robot (took %.2f s, %d collision(s) reported)",t_travel, n_collisions);
title(titlestr)
xlabel("x-axis [m]")
ylabel("y-axis [m]")
axis equal;
hold on;

plot(xcollisions, ycollisions, "x");
plot(xpos, ypos, "Color", "b");

% Limit axes
xlim(xlims);
ylim(ylims);

% Print out information
fprintf("Encountered %d collisions\n", length(xcollisions));
fprintf("Took %f seconds to travel through the warehouse\n", t_travel); % Improvable


