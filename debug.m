clc;
clear;
close all;

% Load Part A trajectory data
load('partA_H_trajectory.mat', 'H_traj');   % handle frame trajectory
load('partA_D_trajectory.mat', 'D_traj');   % door frame trajectory

% Extract the initial transforms
g_H0 = H_traj(:, :, 1);   % {H}_0 (handle in world at t = 0)
g_D0 = D_traj(:, :, 1);   % {D}_0 (door in world at t = 0)

% Define the fixed hand–eye transform g_he from {H} to {e}
R_he = [0 0 -1;
        0 1  0;
        1 0  0];
p_he = [0.1; 0.1; 0];
g_he = [R_he, p_he;
        0 0  0  1];

% Compute the initial end-effector frame {e}_0 in the world:
% {e} = {H} * g_he
g_e0 = g_H0 * g_he;

% Door dimensions (same as Part A)
l = 1.0;          % door width [m]
door_height = 2;  % door height [m]

% Plotting the initial configuration
figure('Name','Initial Door & Frames','Color','w');
axis equal;
view(35, 25);
grid on;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
hold on;

% Plot door shape in world coordinates (all four edges)
door_vertices = [ 0         0           0;
                  l         0           0;
                  l         0    door_height;
                  0         0    door_height;
                  0         0           0 ]';
doorW = g_D0(1:3,1:3) * door_vertices + g_D0(1:3,4);
plot3(doorW(1,:), doorW(2,:), doorW(3,:), ...
      'Color', [0.8 0.4 0.1], 'LineWidth', 2);

% Plot hinge axis (dashed line)
plot3([0 0], [2 0], [0 0], 'k--', 'LineWidth', 1);

% Plot the door frame {D}_0 at its world pose (blue axes)
trplot(g_D0, 'frame','D_0', 'color','b', 'length', 0.3);

% Plot the handle frame {H}_0 at its world pose (red axes)
trplot(g_H0, 'frame','H_0', 'color','r', 'length', 0.3);

% Plot the end-effector frame {e}_0 at its world pose (green axes)
trplot(g_e0, 'frame','e_0', 'color','g', 'length', 0.3);

% Adjust axis limits to clearly see everything
axis([-0.2  1.4   0   2.8   0   2.2]);

% Title
title('Initial Configuration: Door, Handle {H}_0, and End-Effector {e}_0');

hold off;
