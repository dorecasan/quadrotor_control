
% Plotting the 3D trajectories
figure; 
line1 = plot3(RefereneTraj.pxd.Data, RefereneTraj.pyd.Data, RefereneTraj.pzd.Data, 'LineWidth', 1, 'Color', 'k');
hold on;

line2 = plot3(outStatesProposedController.states.px.Data, outStatesProposedController.states.py.Data, outStatesProposedController.states.pz.Data, 'LineWidth', 1, 'Color', 'r');
line3 = plot3(outStatesControllerA.states.px.Data, outStatesControllerA.states.py.Data, outStatesControllerA.states.pz.Data, 'LineWidth', 1, 'Color', 'g');
line4 = plot3(outStatesControllerB.states.px.Data, outStatesControllerB.states.py.Data, outStatesControllerB.states.pz.Data, 'LineWidth', 1, 'Color', 'b');
hold off;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Quadrotor 3D Trajectories');
grid on;
legend('Reference', 'DTLANNC', 'TLMBC', 'NNSMC');
% Customize plot appearance
set(gcf, 'Color', [0.95, 0.95, 0.95]);  % Set figure background color
set(gca, 'Color', [0.9, 0.9, 0.9]);  % Set axes background color
set(gca, 'GridColor', [0.6, 0.6, 0.6]);  % Set grid color
set(gca, 'XMinorGrid', 'on', 'YMinorGrid', 'on', 'ZMinorGrid', 'on');  % Show minor grid lines
set(gca, 'TickDir', 'out');  % Place tick marks outside the plot
set(gca, 'FontName', 'Arial');  % Set font name for axis labels

% Adjust legend properties
legend('Location', 'best');
legend('Box', 'off');

