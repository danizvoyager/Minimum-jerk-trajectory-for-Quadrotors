x=out.response(:,1);
y=out.response(:,2);
z=out.response(:,3);
xd=out.desired(:,1);
yd=out.desired(:,2);
zd=out.desired(:,3);
plot3(x, y, z, 'k', x, y, z,'--b', 'LineWidth', 1.5);
ax = gca; % Get current axes
set(ax, 'XLim', [-5 5], 'YLim', [-5 5], 'ZLim', [0 10]); % Set X and Y limits
% Set axis limits and labels
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
legend('response','desired trajectory')
title('Line with Endpoints');
grid on;