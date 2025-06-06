clc;
clear;
close all

lo = 0.1;          % knob distance from hinge
h  = 0.7;          % knob height
l  = 1.0;          % door width
door_height = 2.0; % door height

T1 = 2; T2 = 3;  T = T1 + T2;
N1 = 100; N2 = 150;

alpha = [tpoly(0,deg2rad(-45),N1)'  tpoly(deg2rad(-45),0,N2)'];   % knob twist/return
beta  = [zeros(1,N1)                 tpoly(0,deg2rad(-30),N2)']; % door swing
n  = numel(alpha);
t  = linspace(0,T,n);
dt = T/(n-1);

% Preallocate trajectory arrays
g_oh_traj = zeros(4,4,n);
g_od_traj = zeros(4,4,n);

T_hinge = transl([0 2 0]);   % hinge at (0,2,0)

figure
for k = 1:n
    T_D0 = T_hinge * trotz(beta(k));            % door frame {D}
    T_HD = transl([l-lo 0 h]) * trotz(-pi/2) * trotx(alpha(k));

    T_H0 = T_D0 * T_HD;                         % knob in world

    % Store transforms into trajectory arrays
    g_od_traj(:,:,k) = T_D0;
    g_oh_traj(:,:,k) = T_H0;

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
save('partA_oh_traj.mat', 'g_oh_traj');
save('partA_od_traj.mat', 'g_od_traj');
