% Author: Boyang Li
% The Hong Kong Polytechnic University
% email: boyang.li@connect.polyu.edu.hk
% Website: https://boyangli.com
% May 2014;

function [] = plot_trans(t,u,z)
figure(1);clf;
set(gcf,'position',[0 0 1600 900]);
subplot(2,2,1)
yyaxis left
plot(t,u(1,:));
xlabel('t (s)')
ylabel('Ft_{cmd} (N)')
yyaxis right
plot(t,u(2,:)*57.3);
ylabel('\theta_{cmd} (бу)')
title('Optimizer Output')

subplot(2,2,2)
plot(t,z(3,:)*57.3);
xlabel('t (s)')
ylabel('\theta (бу)');
title('Pitch Angle')

subplot(2,2,3)
plot(t,z(1:2,:));
xlabel('t (s)')
ylabel('Velocity (m/s)');
legend('$\dot{X}$','$\dot{Z}$');
legend({},'Interpreter','latex')
title('Inertial Velocity')

% plot path with numerical integration
dist_X = cumtrapz(z(1,:)) * (t(end)/150);
dist_Z = cumtrapz(z(2,:)) * (t(end)/150);
subplot(2,2,4)
plot(dist_X, dist_Z);
set(gca,'Ydir','reverse');
axis ([min(dist_X)-0.5, max(dist_X)+1, -2, 1])
xlabel('X_i (m)')
ylabel('Z_i (m)')
title('Trajectory and Attitude')

att_array = zeros(4,11);
att_array(:,1) = [0;0;0;-1];
for i = 1:10
    att_array (:,i+1) = [dist_X(i*15);dist_Z(i*15);-sin(z(3,i*15));-cos(z(3,i*15))];
end
hold on
quiver(att_array(1,:),att_array(2,:),att_array(3,:),att_array(4,:), 0.2,'r');
axis equal;
hold off