% plotSDI.m - Plot results
%
% Plot signals using Simulink's Simulation Data Inspector (SDI)
%
%-List of available signals
%
% The list of signals is made up of the states, inputs, auxilary variables, 
% parameters when they are included in the optimmisation, and other quantities
% relevant for the vehicle simulation like slip angles, slip ratios, or tyre
% forces. In addition, for debugging or other purposes, some constraints can be 
% monitored. Data about the track is also logged into the Data Inspector. 'Custom'
% signals can be created (calculated) and logged in the 'Postprocessing' section of 'MLTP.m'
%
% Signals are retrieved using their identifier (as shown in the example below):
%  STATES
%   ·vx - longitudinal velocity
%   ·vy - lateral velocity
%   ·r - yaw rate
%   ·n - normal distance to the centre line
%   ·eps - relative angle to the tangent of the centre line
%  CONTROL INPUTS
%   ·delta - steering angle (of the wheel)
%   ·Tdrive - total driving torque at the wheels
%   ·Tbrake - total braking torque at the wheels
%  AUX. VARIABLES
%   ·loadTransferX - longitudinal load transfer
%  VEHICLE DATA
%   ·sa_f - slip angle, front tyres
%   ·sa_r - slip angle, rear tyres
%   ·sx_f - slip ratio, front tyres
%   ·sx_r - slip ratio, rear tyres
%   ·fx_f - longitudinal force, front tyres (local coordinates)
%   ·fx_r - longitudinal force, rear tyres (local coordinates)
%   ·fy_f - lateral force, front tyres (local coordinates)
%   ·fy_r - lateral force, rear tyres (local coordinates)
%   ·fz_f - vertical force, front tyres (local coordinates)
%   ·fz_r - vertical force, rear tyres (local coordinates)
%   ·f_drag - aerodynamic drag force
%   ·f_lift - aerodynamic downforce
%  PARAMETERS
%   ·wB - weight distribution
%   ·brkB - brake balance
%  TRACK DATA
%   ·s - distance along centre line
%   ·k - curvature
%   ·x - x coordinate
%   ·k - y coordinate
%  PATH CONSTRAINTS
%   ·BrTh - brake and throttle overlap
%   ·ltx_eq - longitudinal load transfer
%   ·mu_lim_f - friction circle limitation, front tyre
%   ·ltx_eq - friction circle limitation, rear tyre
% 
% *An error might occur if there are existing plots in SDI with a visualization style different from 'time plot'.


%% Plot in SDI

%-Clear previously plotted signals
Simulink.sdi.clearAllSubPlots;

%-Set plot layotu (rows, columns)
Simulink.sdi.setSubPlotLayout(4,3);

%-Select signals to plot
%-states
vxSig = getSignalsByName(data.sdi.run,'vx'); vxSig.plotOnSubPlot(1,1,true);
OmfSig = getSignalsByName(data.sdi.run,'Om_f'); OmfSig.plotOnSubPlot(1,2,true);
OmrSig = getSignalsByName(data.sdi.run,'Om_r'); OmrSig.plotOnSubPlot(1,2,true);
kSig = getSignalsByName(data.sdi.run,'k'); kSig.plotOnSubPlot(1,3,true);
%-inputs
deltaSig = getSignalsByName(data.sdi.run,'delta'); deltaSig.plotOnSubPlot(2,1,true);
fdriveSig = getSignalsByName(data.sdi.run,'T_drive'); fdriveSig.plotOnSubPlot(3,1,true);
fbrakeSig = getSignalsByName(data.sdi.run,'T_brake'); fbrakeSig.plotOnSubPlot(3,1,true);
%-vehicle
%%-tyres
sxfSig = getSignalsByName(data.sdi.run,'sx_f'); sxfSig.plotOnSubPlot(3,2,true);
sxrSig = getSignalsByName(data.sdi.run,'sx_r'); sxrSig.plotOnSubPlot(3,2,true);
safSig = getSignalsByName(data.sdi.run,'sa_f'); safSig.plotOnSubPlot(2,2,true);
sarSig = getSignalsByName(data.sdi.run,'sa_r'); sarSig.plotOnSubPlot(2,2,true);
fxfSig = getSignalsByName(data.sdi.run,'fx_f'); fxfSig.plotOnSubPlot(3,3,true);
fxrSig = getSignalsByName(data.sdi.run,'fx_r'); fxrSig.plotOnSubPlot(3,3,true);
fyfSig = getSignalsByName(data.sdi.run,'fy_f'); fyfSig.plotOnSubPlot(2,3,true);
fyrSig = getSignalsByName(data.sdi.run,'fy_r'); fyrSig.plotOnSubPlot(2,3,true);
fzfSig = getSignalsByName(data.sdi.run,'fz_f'); fzfSig.plotOnSubPlot(4,3,true);
fzrSig = getSignalsByName(data.sdi.run,'fz_r'); fzrSig.plotOnSubPlot(4,3,true);
%%-load transfer
ltxSig = getSignalsByName(data.sdi.run,'loadTransferX'); ltxSig.plotOnSubPlot(4,2,true);
%%-aerodynamic forces
f_dragSig = getSignalsByName(data.sdi.run,'f_drag'); f_dragSig.plotOnSubPlot(4,1,true);
f_dragSig = getSignalsByName(data.sdi.run,'f_drag'); f_dragSig.plotOnSubPlot(4,1,true);

% Open SDI (Simulation Data Inspector)
Simulink.sdi.view

