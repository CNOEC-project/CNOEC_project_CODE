function [z_prime,F,t_prime]=vehicle_dynamics(z,u,chassis,tyre,rho)
% VEHICLE_DYNAMICS - Nonlinear dynamic model of a rear-wheel drive road vehicle with 7 states:
% longitudinal velocity, transversal velocity, yaw rate, transversal
% displacement, front wheel angular velocity, rear wheel angular velocity,
% heading angle. 
% The model has 3 inputs: the driving torque, the braking torque and the
% steering angle.
%
% Inputs:   
%           z               (model state)
%           u               (driving torque, braking torque, steering angle)          
%           chassis         (chassis parameters)
%           tyre            (tyre parameters)
%           rho             (local circuit curvature)
%
% Outputs:  t_prime         (dt/ds)
%           z_prime         (dz/ds)
%           F               (longitudinal and lateral forces)

%% Read parameters
% Chassis parameters
m       =       chassis.m;           % vehicle mass (kg)
lf       =      chassis.lf;          % distance between center of gravity and front axle (m)
lr       =      chassis.lr;          % distance between center of gravity and rear axle (m)
h       =       chassis.h;           % distance between center of gravity and road (m)
wv      =       chassis.wv;          % vehicle width (m)
Izz     =       chassis.Izz;         % vehicle moment of inertia around z axis(kg*m^2)
rho_air =       chassis.rho_air;     % air density (kg/m^3)
A_f     =       chassis.A_f;         % vehicle front area (m^2)
Cd      =       chassis.Cd;          % aerodynamic drag coefficient (-)
Cl      =       chassis.Cl;          % aerodynamic lift coefficient (-)
brake_dist =    chassis.brake_dist;  % Brake distribution (-)

lv      =       lr+lf;               % vehicle total length (m)
weight_dist =   lr/lv;               % vehicle weight distribution (-)
g       =       9.81;                % gravity acceleration (m/s^2)

% Tyre parameters
rw      =       tyre.rw;    % wheel radius (m)
Jw      =       tyre.Jw;    % wheel rotational inertia (kg*m^2)
f_rol   =       tyre.f_rol;    % rolling resistance adimensional coefficient (-)
mu0_x   =       tyre.mu0_x;    % Maximum longitudinal friction coefficient with static load (-)
mu0_y   =       tyre.mu0_y;    % Maximum lateral friction coefficient with nominal vertical load (-)
bx      =       tyre.bx;    % Pacejka's magic formula bx coefficient for longitudinal force (-)
cx      =       tyre.cx;    % Pacejka's magic formula cx coefficient for longitudinal force (-)
ex      =       tyre.ex;    % Pacejka's magic formula ex coefficient for longitudinal force (-)
by      =       tyre.by;    % Pacejka's magic formula by coefficient for lateral force (-)
cy      =       tyre.cy;    % Pacejka's magic formula cy coefficient for lateral force (-)
ey      =       tyre.ey;    % Pacejka's magic formula ey coefficient for lateral force (-)
Fz0     =       tyre.Fz0;    % Pacejka's magic formula nominal vertical load (N)
pD2     =       tyre.pD2;    % Pacejka's magic formula tyre load sensitivity coefficient (-)

%% Read states and inputs
% States
vx       =       z(1,1);     % body x velocity (m/s)
vy       =       z(2,1);     % body y velocity (m/s)
omega_z  =       z(3,1);     % yaw rate (rad/s)
n        =       z(4,1);     % transversal displacement (m)
epsi     =       z(5,1);     % heading angle (rad)       
omega_f  =       z(6,1);     % front wheel rotational speed (rad/s)
omega_r  =       z(7,1);     % rear wheel rotational speed (rad/s)

n_states =      length(z);  % number of states

% Inputs
T_drive             =       u(1,1);     % rear driving torque (Nm)
T_brake             =       u(2,1);     % rear braking torque (Nm)
delta               =       u(3,1);     % steering angle (rad)

% Wheel torque considering the brake distribution
T_f = T_brake*brake_dist;               % Torque on the front wheel (Nm)
T_r = T_drive + T_brake*(1-brake_dist); % Torque on the rear wheel (Nm)

%% Compute the forces acting on the vehicle
[F_drag,F_lift,F_roll_f,F_roll_r,Fz_f,Fz_r,Fx_f,Fx_r,Fy_f,Fy_r] = forces_computation(z,u,chassis,tyre);

F = [F_drag,F_lift,F_roll_f,F_roll_r,Fz_f,Fz_r,Fx_f,Fx_r,Fy_f,Fy_r]';

%% Dynamic equations
z_prime=zeros(n_states,1);

% Model equations
t_prime      =   (1-n*rho)/(vx*cos(epsi)-vy*sin(epsi));                                           % dt/ds 

z_prime(1,1)  =   ((Fx_r+Fx_f*cos(delta)-Fy_f*sin(delta)-F_drag)/m+omega_z*vy)*t_prime;           % vx_prime
z_prime(2,1)  =   ((Fy_r+Fx_f*sin(delta)+Fy_f*cos(delta))/m-omega_z*vx)*t_prime;                  % vy_prime
z_prime(3,1)  =   ((lf*Fy_f*cos(delta)+lf*Fx_f*sin(delta)-lr*Fy_r)/Izz)*t_prime;                  % omega_prime
z_prime(4,1)  =   (vx*sin(epsi)+vy*cos(epsi))*t_prime;                                            % n_prime
z_prime(5,1)  =   (omega_z*t_prime-rho);                                                          % alfa_prime
z_prime(6,1)  =   ((T_f - Fx_f*rw)/Jw)*t_prime;                                                     % omega_f_prime
z_prime(7,1)  =   ((T_r - Fx_r*rw)/Jw)*t_prime;                                                     % omega_r_prime
