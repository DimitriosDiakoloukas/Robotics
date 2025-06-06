clc; clear; close all;

ur10 = ur10robot();
T = 5;
dt = 0.01;
time = 0:dt:T;
N = length(time);

q0 = [-1.7752 -1.1823 0.9674 0.2149 1.3664 1.5708];
q_traj = zeros(N, 6);
qd_traj = zeros(N, 6);

load('partA_oh_traj.mat','g_oh_traj');
load('partA_od_traj.mat','g_od_traj');

R_he = [0 0 -1;
         0 1 0;
         1 0 0];

p_he = [0.1; 0.1; 0];

g_he = [R_he, p_he;
        0 0 0 1];

g_eh = inv(g_he);

q = q0';
q_traj(1,:) = q0;
g_oh = g_oh_traj(:,:,1);
g_oe = g_oh * g_he;

for k = 2:N
    g_oh = g_oh_traj(:,:,k);
    g_oe_des = g_oh * g_he;

    g_oe_curr = ur10.fkine(q);

    deltaT = tr2delta(g_oe_curr.T, g_oe_des);

    J = ur10.jacobe(q);

    q_dot = pinv(J) * (deltaT / dt);

    q = q + q_dot * dt;

    q_traj(k,:) = q';
    qd_traj(k,:) = q_dot';
end

figure('Name','UR10 Animation');
axis([-1 2 -0.5 3 0 1.5]);
view(3); grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on;

trplot(eye(4), 'frame', '0', 'length', 0.2, 'color', 'k');
plot3([-1 2], [0 0], [0 0], 'k--');
plot3([0 0], [-0.5 3], [0 0], 'k--');

ur10.plot(q_traj(1,:), ...
    'workspace', [-1 2 -0.5 3 0 1.5], ...
    'view', [45 30]);

g_e_start = ur10.fkine(q_traj(1,:));
g_h_start = g_oh_traj(:,:,1);
g_d_start = g_od_traj(:,:,1);

trplot(g_d_start, 'frame', 'd_0', 'color', 'b', 'length', 0.2);
trplot(g_e_start.T, 'frame', 'e_0', 'color', 'r', 'length', 0.2);
trplot(g_h_start, 'frame', 'H_0', 'color', 'r', 'length', 0.2);

step_interval = 4;
num_frames = floor(N / step_interval);
dt_anim = T / num_frames;

for j = 1:num_frames
    i = (j - 1) * step_interval + 1;

    ur10.animate(q_traj(i,:));

    g_e = ur10.fkine(q_traj(i,:));
    g_h = g_oh_traj(:,:,i);
    g_d = g_od_traj(:,:,i);

    if exist('h_e','var'), delete(h_e); end
    if exist('h_h','var'), delete(h_h); end
    if exist('h_d','var'), delete(h_d); end
    if exist('h_handle','var'), delete(h_handle); end
    if exist('h_door','var'), delete(h_door); end

    h_e = trplot(g_e.T, 'frame', 'e', 'color', 'g', 'length', 0.2);
    h_h = trplot(g_h, 'frame', 'H', 'color', 'g', 'length', 0.2);
    h_d = trplot(g_d, 'frame', 'd', 'color', 'g', 'length', 0.2);

    h_handle = plotHandle(g_h, 0.02, 0.15);
    h_door = plotDoor(g_d, 1, 2.2);

    drawnow;
end

figure('Name','Joint Positions');
plot(time, q_traj, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Joint angles [rad]');
legend('q1','q2','q3','q4','q5','q6');
grid on; title('Joint Position Trajectories');

figure('Name','Joint Velocities');
plot(time, qd_traj, 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Joint velocities [rad/s]');
legend('q̇1','q̇2','q̇3','q̇4','q̇5','q̇6');
grid on; title('Joint Velocity Trajectories');

p_oe_traj = zeros(N,3);
q_oe_traj = zeros(N,4);

for k = 1:N
    g_oe = ur10.fkine(q_traj(k,:));
    p_oe_traj(k,:) = transl(g_oe)';
    q_oe_traj(k,:) = UnitQuaternion(g_oe).double;
end

figure('Name','End-Effector Position Trajectory');
plot3(p_oe_traj(:,1), p_oe_traj(:,2), p_oe_traj(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End-Effector Position Trajectory');
legend('Trajectory');

figure('Name','End-Effector Orientation (Quaternion)');
plot(time, q_oe_traj, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3');
grid on;
title('End-Effector Orientation in Unit Quaternion');

p_eh_traj = zeros(N, 3);
q_eh_traj = zeros(N, 4);

for k = 1:N
    g_oh = g_oh_traj(:,:,k);
    g_oe = ur10.fkine(q_traj(k,:));
    g_eh = inv(g_oh) * g_oe.T;

    p_eh_traj(k,:) = transl(g_eh)';
    q_eh_traj(k,:) = UnitQuaternion(g_eh).double;
end

figure('Name','Relative Position: End-Effector w.r.t. Handle');
plot3(p_eh_traj(:,1), p_eh_traj(:,2), p_eh_traj(:,3), 'LineWidth', 2);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Relative Position of End-Effector w.r.t. Handle');

figure('Name','Relative Orientation: End-Effector w.r.t. Handle');
plot(time, q_eh_traj, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Quaternion Components');
legend('q_0','q_1','q_2','q_3');
grid on;
title('Relative Orientation of End-Effector w.r.t. Handle');

function h = plotHandle(g_oh, radius, length)
    [X, Y, Z] = cylinder(radius);
    Z = Z * length;
    Z = Z - length/2;

    R = [1 0 0;
         0 0 1;
         0 -1 0];

    n = numel(X);
    points = [X(:)'; Y(:)'; Z(:)'];
    points = R * points;

    points = [points; ones(1, n)];

    transformed = g_oh * points;
    Xt = reshape(transformed(1,:), size(X));
    Yt = reshape(transformed(2,:), size(Y));
    Zt = reshape(transformed(3,:), size(Z));

    h = surf(Xt, Yt, Zt, ...
        'FaceColor', [0.5 0.5 0.5], ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 1);
end

function h = plotDoor(g_od, width, height)
    corners = [0 width width 0;
               0 0 0 0;
               0 0 height height;
               1 1 1 1];

    transformed = g_od * corners;

    X = transformed(1,:);
    Y = transformed(2,:);
    Z = transformed(3,:);

    h = patch(X, Y, Z, [0 0 0], 'FaceAlpha', 1);
end
