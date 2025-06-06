clc;
clear;
close all;

% UR10 setup
ur10 = ur10robot();

% Total motion time (5 s)
T = 5;

% Load Part A trajectories (handle frame {H} and door frame {D})
load('partA_oh_traj.mat','g_oh_traj');
load('partA_od_traj.mat','g_od_traj');

% Number of time steps from Part A (501 frames (N1 = 201 and N2 = 300)) and
% now dt is T / (n - 1) = 5 sec / (501 - 1) = 0.01 sec sampling rate.
n  = size(g_oh_traj, 3);
dt = T / (n - 1);
fprintf("Sampling rate at: %.2f", dt);
time = linspace(0, T, n);

% Preallocate joint trajectories
q0      = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];
q_traj  = zeros(n, 6);
qd_traj = zeros(n, 6);

% Hand–eye transform (fixed between {H} and {e})
R_he = [0 0 -1;
        0 1  0;
        1 0  0];
p_he = [0.1; 0.1; 0];
g_he  = [R_he, p_he;
         0 0  0  1];
g_eh  = inv(g_he);

% Initialize with q0
q = q0';
q_traj(1, :) = q0;

% Inverse-kinematics: {e} → {H}⋅g_he
for k = 2:n
    g_H = g_oh_traj(:, :, k);      % handle frame in world
    g_e_des = g_H * g_he;          % desired end–effector pose

    g_e_curr = ur10.fkine(q);      % current EE pose
    deltaT = tr2delta(g_e_curr.T, g_e_des);

    J = ur10.jacobe(q);
    q_dot = pinv(J) * (deltaT / dt);

    q = q + q_dot * dt;
    q_traj(k, :)  = q';
    qd_traj(k, :) = q_dot';
end

% Door dimensions (same as Part A)
l = 1.0;          % door width
door_height = 2;  % door height

figure('Name','UR10 Animation');
axis([-1 2   0 3   0 2.5]);
view(3); grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
hold on;

% Base frame {0} and reference lines
trplot(eye(4), 'length', 0.2, 'color','k');
plot3([-1 2], [0 0], [0 0], 'k--');
plot3([0 0], [-0.5 3], [0 0], 'k--');

% Plot initial UR10 pose
ur10.plot(q_traj(1, :), ...
    'workspace', [-1 2  -0.5 3  0 1.5], ...
    'view', [45 30]);

% Indices to plot: every 4th step + final step
step_interval = 4;
idx = 1:step_interval:n;
if idx(end) ~= n
    idx = [idx, n];
end

for k_idx = 1:length(idx)
    i = idx(k_idx);

    ur10.animate(q_traj(i, :));  % robot at step i

    g_e = ur10.fkine(q_traj(i, :));   % end–effector
    g_H = g_oh_traj(:, :, i);         % handle
    g_D = g_od_traj(:, :, i);         % door

    % Remove previous plots
    if exist('h_door','var'), delete(h_door); end
    if exist('h_hinge','var'), delete(h_hinge); end
    if exist('h_e','var'), delete(h_e); end
    if exist('h_H','var'), delete(h_H); end
    if exist('h_D','var'), delete(h_D); end

    % Plot frames with updated labels
    h_D = trplot(g_D, 'frame','D','color','b','length',0.5);
    h_e = trplot(g_e.T, 'frame','e','color','g','length',0.5);
    h_H = trplot(g_H, 'frame','H','color','r','length',0.5);

    % Plot door outline (all four edges)
    door = [ 0      0        0;
             l      0        0;
             l      0   door_height;
             0      0   door_height;
             0      0        0 ]';
    doorW = g_D(1:3,1:3) * door + g_D(1:3,4);
    h_door = plot3(doorW(1,:), doorW(2,:), doorW(3,:), ...
                   'Color',[.8 .4 .1], 'LineWidth', 2);

    % Plot hinge axis
    h_hinge = plot3([0 0], [2 0], [0 0], 'k--','LineWidth',1);

    % Improved live-plot title
    title(sprintf('Animation Time: %.2f seconds', time(i)));

    drawnow;

    % Real-time pause
    if k_idx < length(idx)
        pause((idx(k_idx+1) - i) * dt);
    end
end

% ------------------------------------------------------------------------
% Plot joint positions over time with new title
% ------------------------------------------------------------------------
figure('Name','Joint Positions Over Time');
plot(time, q_traj, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Joint Angles [rad]');
legend('q_1','q_2','q_3','q_4','q_5','q_6','Location','best');
grid on; title('UR10 Joint Angles vs. Time');

% ------------------------------------------------------------------------
% Plot joint velocities over time with new title
% ------------------------------------------------------------------------
figure('Name','Joint Velocities Over Time');
plot(time, qd_traj, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Joint Velocities [rad/s]');
legend('ẋ_1','ẋ_2','ẋ_3','ẋ_4','ẋ_5','ẋ_6','Location','best');
grid on; title('UR10 Joint Velocity Profiles');

% ------------------------------------------------------------------------
% End–effector position trajectory
% ------------------------------------------------------------------------
p_e_traj = zeros(n, 3);
q_e_traj = zeros(n, 4);
for k = 1:n
    g_e = ur10.fkine(q_traj(k, :));
    p_e_traj(k, :) = transl(g_e)';
    q_e_traj(k, :) = UnitQuaternion(g_e).double;
end

figure('Name','End–Effector Position Path');
plot3(p_e_traj(:,1), p_e_traj(:,2), p_e_traj(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('3D Path of End–Effector');

% ------------------------------------------------------------------------
% End–effector orientation (quaternion) over time
% ------------------------------------------------------------------------
figure('Name','End–Effector Orientation');
plot(time, q_e_traj, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3','Location','best');
grid on; title('End–Effector Quaternion vs. Time');

% ------------------------------------------------------------------------
% Relative position of end–effector w.r.t. handle
% ------------------------------------------------------------------------
p_eh_traj = zeros(n, 3);
q_eh_traj = zeros(n, 4);
for k = 1:n
    g_H = g_oh_traj(:, :, k);
    g_e = ur10.fkine(q_traj(k, :));
    g_eh = inv(g_H) * g_e.T;
    p_eh_traj(k, :) = transl(g_eh)';
    q_eh_traj(k, :) = UnitQuaternion(g_eh).double;
end

figure('Name','EE Relative Position to Handle');
plot3(p_eh_traj(:,1), p_eh_traj(:,2), p_eh_traj(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Relative 3D Position: End–Effector w.r.t. Handle');

% ------------------------------------------------------------------------
% Relative orientation of end–effector w.r.t. handle over time
% ------------------------------------------------------------------------
figure('Name','EE Relative Orientation');
plot(time, q_eh_traj, 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3','Location','best');
grid on; title('Relative Quaternion: End–Effector w.r.t. Handle');


