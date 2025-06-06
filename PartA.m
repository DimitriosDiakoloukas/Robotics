clc;
clear;
close all

lo = 0.1;          % knob distance from hinge
h  = 0.7;          % knob height
l  = 1.0;          % door width
door_height = 2.0; % door height

T1 = 2; T2 = 3;  T = T1 + T2;
N1 = 201; N2 = 300;

alpha = [tpoly(0,deg2rad(-45),N1)'  tpoly(deg2rad(-45),0,N2)'];   % knob twist/return
beta  = [zeros(1,N1)                 tpoly(0,deg2rad(-30),N2)']; % door swing
n  = numel(alpha);
t  = linspace(0,T,n);
dt = T/(n-1);
fprintf("Sampling rate at: %.2f", dt);

% Preallocate trajectory arrays
H_traj = zeros(4,4,n);
D_traj = zeros(4,4,n);

T_hinge = transl([0 2 0]);   % hinge at (0,2,0)

figure
for k = 1:n
    T_D0 = T_hinge * trotz(beta(k));            % door frame {D}
    T_HD = transl([l-lo 0 h]) * trotz(-pi/2) * trotx(alpha(k));

    T_H0 = T_D0 * T_HD;                         % knob in world

    % Store transforms into trajectory arrays
    D_traj(:,:,k) = T_D0;
    H_traj(:,:,k) = T_H0;

    clf; axis equal; grid on; view(35,25); hold on
    xlabel X; ylabel Y; zlabel Z

    trplot(eye(4),'frame','0','color','k','length',0.5)
    trplot(T_D0 ,'frame','D','color','b','length',0.5)
    trplot(T_H0 ,'frame','H','color','r','length',0.5)

    door = [0 0 0;
            l 0 0;
            l 0 door_height;
            0 0 door_height;
            0 0 0]';               
    doorW = T_D0(1:3,1:3)*door + T_D0(1:3,4);
    plot3(doorW(1,:),doorW(2,:),doorW(3,:),...
          'Color',[.8 .4 .1],'LineWidth',2)

    plot3([0 0],[2 0],[0 0],'k--','LineWidth',1) % hinge axis
    axis([-0.2 1.4   0 2.8   0 2.2])
    title(sprintf('Time: %.2f s',t(k)))
    drawnow; pause(dt);
end

% Save trajectories for Part B
save('partA_H_trajectory.mat', 'H_traj');
save('partA_D_trajectory.mat', 'D_traj');

% --- Additional Plots for Part A ---

% Extract handle position and quaternion from H_traj
p_h_traj = zeros(n, 3);   % handle positions
q_h_traj = zeros(n, 4);   % handle quaternions

for k = 1:n
    gH = H_traj(:,:,k);
    p_h_traj(k, :) = transl(gH)';                     % [x, y, z]
    q_h_traj(k, :) = UnitQuaternion(gH).double;       % [q0, q1, q2, q3]
end

% Plot handle position components vs. time
figure('Name','Handle Position vs Time','Color','w');
plot(t, p_h_traj(:,1), 'LineWidth', 1.5); hold on;
plot(t, p_h_traj(:,2), 'LineWidth', 1.5);
plot(t, p_h_traj(:,3), 'LineWidth', 1.5);
grid on; axis tight;
xlabel('Time [s]');
ylabel('Handle Position [m]');
legend('x_h','y_h','z_h','Location','best');
title('Handle Position Components Over Time');

% Plot handle quaternion components vs. time
figure('Name','Handle Orientation (Quaternion) vs Time','Color','w');
plot(t, q_h_traj(:,1), 'LineWidth', 1.5); hold on;
plot(t, q_h_traj(:,2), 'LineWidth', 1.5);
plot(t, q_h_traj(:,3), 'LineWidth', 1.5);
plot(t, q_h_traj(:,4), 'LineWidth', 1.5);
grid on; axis tight;
xlabel('Time [s]');
ylabel('Quaternion Component');
legend('q_0','q_1','q_2','q_3','Location','best');
title('Handle Orientation (Unit Quaternion) Over Time');

% Plot a (knob twist) and b (door swing) vs. time
figure('Name','Knob Twist & Door Swing Profiles','Color','w');
yyaxis left
plot(t, rad2deg(alpha), 'LineWidth', 1.5);
ylabel('Knob Twist \alpha [deg]');
yyaxis right
plot(t, rad2deg(beta), '--', 'LineWidth', 1.5);
ylabel('Door Swing \beta [deg]');
grid on; axis tight;
xlabel('Time [s]');
legend('\alpha (knob twist)','\beta (door swing)','Location','best');
title('Knob Twist and Door Swing vs Time');