
figure(1);
ax1 = subplot(4,1,1);
plotFigure('X (m)',outStatesControllerA.states.px.Time, ...
    RefereneTraj.pxd.Data,...
    outStatesProposedController.states.px.Data,...
    outStatesControllerA.states.px.Data, ...
    outStatesControllerB.states.px.Data,ax1);
ax2 = subplot(4,1,2);
plotFigure('Y (m)',outStatesControllerA.states.py.Time,RefereneTraj.pyd.Data, ... 
    outStatesProposedController.states.py.Data,...
    outStatesControllerA.states.py.Data, ...
    outStatesControllerB.states.py.Data,ax2);
ax3 = subplot(4,1,3);
plotFigure('Z (m)',outStatesControllerA.states.pz.Time,RefereneTraj.pzd.Data, ...
    outStatesProposedController.states.pz.Data,...
    outStatesControllerA.states.pz.Data,...
    outStatesControllerB.states.pz.Data,ax3);
ax4 = subplot(4,1,4);
plotFigure('psi (rad)',outStatesControllerA.states.psi.Time, ...
    RefereneTraj.psid.Data, ...
    outStatesProposedController.states.psi.Data,...
    outStatesControllerA.states.psi.Data,...
    outStatesControllerB.states.psi.Data,ax4);

figure(2);
ax1 = subplot(4,1,1);
plotFigure('eX (m)',outStatesControllerA.states.px.Time,RefereneTraj.pxd.Data - RefereneTraj.pxd.Data, ...
    outStatesProposedController.states.px.Data - RefereneTraj.pxd.Data,...
    outStatesControllerA.states.px.Data - RefereneTraj.pxd.Data, ...
    outStatesControllerB.states.px.Data - RefereneTraj.pxd.Data,ax1);
ax2 = subplot(4,1,2);
plotFigure('eY (m)',outStatesControllerA.states.py.Time,RefereneTraj.pyd.Data - RefereneTraj.pyd.Data, ... 
    outStatesProposedController.states.py.Data- RefereneTraj.pyd.Data,...
    outStatesControllerA.states.py.Data- RefereneTraj.pyd.Data, ...
    outStatesControllerB.states.py.Data- RefereneTraj.pyd.Data,ax2);
ax3 = subplot(4,1,3);
plotFigure('eZ (m)',outStatesControllerA.states.pz.Time,RefereneTraj.pzd.Data-RefereneTraj.pzd.Data, ...
    outStatesProposedController.states.pz.Data-RefereneTraj.pzd.Data,...
    outStatesControllerA.states.pz.Data-RefereneTraj.pzd.Data,...
    outStatesControllerB.states.pz.Data-RefereneTraj.pzd.Data,ax3);
ax4 = subplot(4,1,4);
plotFigure('epsi (rad)',outStatesControllerA.states.psi.Time,RefereneTraj.psid.Data - RefereneTraj.psid.Data ...
    ,outStatesProposedController.states.psi.Data- RefereneTraj.psid.Data,...
    outStatesControllerA.states.psi.Data- RefereneTraj.psid.Data,...
    outStatesControllerB.states.psi.Data - RefereneTraj.psid.Data,ax4);

% figure(3);
% ax1 = subplot(4,1,1);
% plotFigure('F (N)',outStatesControllerA.states.px.Time, ...
%     outStatesProposedController.controlSignals.F.Data,...
%     outStatesControllerA.controlSignals.F.Data, ...
%     outStatesControllerB.controlSignals.F.Data,ax1);
% ax2 = subplot(4,1,2);
% plotFigure('tau1 (Nm)',outStatesControllerA.states.py.Time, ...
%     outStatesProposedController.controlSignals.tau1.Data,...
%     outStatesControllerA.controlSignals.tau1.Data, ...
%     outStatesControllerB.controlSignals.tau1.Data,ax2);
% ax3 = subplot(4,1,3);
% plotFigure('tau2 (Nm)',outStatesControllerA.states.pz.Time, ...
%     outStatesProposedController.controlSignals.tau2.Data,...
%     outStatesControllerA.controlSignals.tau2.Data,...
%     outStatesControllerB.controlSignals.tau2.Data,ax3);
% ax4 = subplot(4,1,4);
% plotFigure('tau3 (Nm)',outStatesControllerA.states.psi.Time, ...
%     outStatesProposedController.controlSignals.tau3.Data,...
%     outStatesControllerA.controlSignals.tau3.Data,...
%     outStatesControllerB.controlSignals.tau3.Data,ax4);
