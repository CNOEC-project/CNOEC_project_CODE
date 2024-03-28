%% Initial commands
clear all
close all
clc

%% Model parameters

m       =       1715;               % vehicle mass (kg)
d       =       0.15;               % arbitrarly selected
a       =       1.07;               % distance between center of gravity and front axle (m)
b       =       1.47;               % distance between center of gravity and rear axle (m)
Izz     =       2700;               % vehicle moment of inertia (kg*m^2)
h       =       0.42;               % arbitrarly selected
Ixz     =       -50;                % arbitrarly selected
dr      =       0.789;              % arbitrarly selected
df      =       0.751;              % arbitrarly selected
rw      =       0.303;              % wheel radius (m)      
kt      =       0;
kb      =       0.5;
Cf      =       62;            
Cr      =       52;              
chi     =       0;               % arbitrarly selected
sigma   =       0;                % arbitrarly selected
g       =       9.81;

th=[m;d;a;b;Izz;h;Ixz;dr;df;rw;kt;kb;Cf;Cr;chi;sigma;g];

%% Simulation: Initial state
w       =       0;          % displacement transverse to the central line (m)
mu       =      0;          % local heading angle (rad)
vx      =       10;          % body x velocity (m/s)
vy    =         0;          %  body y velocity (m/s)
r       =       0;          % yaw rate (rad/s)
delta   =       0;        % steering angle(rad)
psi     =       0;          % yaw angle (rad)

z0     =       [w;mu;vx;vy;r;delta;psi];

%% Simulation: step amplitude
u_delta_step    =   10*pi/180;
ut_step         =   0.5;
ub_step         =   0;

% 
% % Simulation with Simulink
% Time integration parameters
% Ts_slk      =       1e-1;               % sampling time (s)
% Tend_slk    =       200;                % final time (s)
% 
% Run simulation
% tic
% sim('vehicle_sim',Tend_slk)
% t_sim=toc;
% [t_FFD t_RK2 t_o45 t_sim]
% Plot the results
% figure(1),plot(zout_slk.signals.values(:,1),zout_slk.signals.values(:,2)),grid on, hold on,xlabel('X (m)'),ylabel('Y (m)')
% figure(2),plot(zout_slk.time,zout_slk.signals.values(:,3)*3.6),grid on, hold on,xlabel('Time (s)'),ylabel('Longitudinal speed (km/h)')
% figure(3),plot(zout_slk.time,zout_slk.signals.values(:,4)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Sideslip angle (deg)')
% figure(4),plot(zout_slk.time,zout_slk.signals.values(:,5)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw angle (deg)')
% figure(5),plot(zout_slk.time,zout_slk.signals.values(:,6)*180/pi),grid on, hold on,xlabel('Time (s)'),ylabel('Yaw rate (deg/s)')
% figure(6),plot(zout_slk.time,Fout_slk.signals.values(:,1)),grid on, hold on,xlabel('Time (s)'),ylabel('Front lateral force (N)')
% figure(7),plot(zout_slk.time,Fout_slk.signals.values(:,2)),grid on, hold on,xlabel('Time (s)'),ylabel('Rear lateral force (N)')
% figure(8),plot(zout_slk.time,zout_slk.signals.values(:,6).*zout_slk.signals.values(:,3)/9.81),grid on, hold on,xlabel('Time (s)'),ylabel('Lateral acceleration (g)')
% % figure(9),plot(zout_slk.signals.values(:,6).*zout_slk.signals.values(:,3),uout_slk.signals.values(:,2)*180/pi),grid on, hold on,xlabel('Lateral acceleration (m/s^2)'),ylabel('Steering angle (deg)')
% 
% % figure(1),legend('Forward Euler','RK2','ode45','simulink')
% % figure(2),legend('Forward Euler','RK2','ode45','simulink')
% % figure(3),legend('Forward Euler','RK2','ode45','simulink')
% % figure(4),legend('Forward Euler','RK2','ode45','simulink')
% % figure(5),legend('Forward Euler','RK2','ode45','simulink')
% % figure(6),legend('Forward Euler','RK2','ode45','simulink')
% % figure(7),legend('Forward Euler','RK2','ode45','simulink')
% % figure(8),legend('Forward Euler','RK2','ode45','simulink')
% % % figure(9),legend('Forward Euler','RK2','ode45','simulink')