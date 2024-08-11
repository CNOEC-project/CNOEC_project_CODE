function [F_drag,F_lift,F_roll_f,F_roll_r,Fz_f,Fz_r,Fx_f,Fx_r,Fy_f,Fy_r] = forces_computation(z,u,chassis,tyre)

%FORCES_COMPUTATION Computes the force acting on the vehicle at the current
% iteration.
% We consider 10 forces acting simultaneously on the vehicle: 
% - Drag and lift aerodynamic forces
% - Front and rear rolling resistances
% - Front and rear vertical load
% - Front and rear longitudinal tyre-road forces
% - Front and rear lateral tyre-road forces

%% Read parameters
% Chassis parameters
m       =       chassis.m;           % vehicle mass (kg)
lf       =      chassis.lf;     % distance between center of gravity and front axle (m)
lr       =      chassis.lr;     % distance between center of gravity and rear axle (m)
h       =       chassis.h;     % distance between center of gravity and road (m)
wv      =       chassis.wv;     % vehicle width (m)
Izz     =       chassis.Izz;     % vehicle moment of inertia around z axis(kg*m^2)
rho_air =       chassis.rho_air;     % air density (kg/m^3)
A_f     =       chassis.A_f;     % vehicle front area (m^2)
Cd      =       chassis.Cd;     % aerodynamic drag coefficient (-)
Cl      =       chassis.Cl;    % aerodynamic lift coefficient (-)
brake_dist =    chassis.brake_dist;    % Brake distribution (-)

lv      =       lr+lf;               % vehicle total length (m)
weight_dist =   lr/lv;               % vehicle weight distribution (-)
g       =       9.81;                % gravity acceleration (m/s^2)

% Tyre parameters
rw      =       tyre.rw;    % wheel radius (m)
Jw      =       tyre.Jw;    % wheel rotational inertia (kg*m^2)
f_rol   =       tyre.f_rol; % rolling resistance adimensional coefficient (-)
mu0_x   =       tyre.mu0_x; % Maximum longitudinal friction coefficient with static load (-)
mu0_y   =       tyre.mu0_y; % Maximum lateral friction coefficient with nominal vertical load (-)
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

%% Drag and lift aerodynamic forces
F_drag = 0.5*rho_air*Cd*A_f*vx^2;       % Drag aerodynamic force (Nm)
F_lift = 0.5*rho_air*Cl*A_f*vx^2;       % Lift aerodynamic force - positive if pointing downwards (Nm)

%% Front and rear rolling resistances
% F_roll_f = f_rol*m*g*lr/lv;             % Rolling resistance on the front wheel (Nm)
% F_roll_r = f_rol*m*g*lf/lv;             % Rolling resistance on the rear wheel (Nm)

F_roll_f = 0;             % Rolling resistance on the front wheel (Nm)
F_roll_r = 0;             % Rolling resistance on the rear wheel (Nm)

%% Front and rear vertical loads (neglect load transfer)
Fz_f = m*g*lr/lv + F_lift/2;                                % Front vertical load (N)
Fz_r = m*g*lf/lv + F_lift/2;                                % Rear vertical load (N)

%% Front and rear longitudinal tyre-road forces
lambda_f = delta-atan((lf*omega_z+vy)/vx);                      % Front wheel longitudinal slip (-)
lambda_r = atan((lr*omega_z-vy)/vx);                            % Rear wheel longitudinal slip (-)

mu_f = mu0_x + pD2*(Fz_f-Fz0)/Fz0;                              % Current front tyre friction coefficient (-) 
mu_r = mu0_y + pD2*(Fz_r-Fz0)/Fz0;                              % Current rear tyre friction coefficient (-) 

Dx_f = mu_f*Fz_f;                                               % Pacejka's variable friction-dependent force term - front wheel (N)
Dx_r = mu_r*Fz_r;                                               % Pacejka's variable friction-dependent force term - rear wheel (N)

vx_f     = sqrt((vy+lf*omega_z)^2 + vx^2)*cos(lambda_f);        % Front wheel longitudinal speed (m/s)
vx_r     = sqrt((vy-lf*omega_z)^2 + vx^2)*cos(lambda_r);        % Rear wheel longitudinal speed (m/s)

sx_f = (rw*omega_f - vx_f)/vx_f;                                % Front longitudinal slip ratio (-)
sx_r = (rw*omega_r - vx_r)/vx_r;                                % Rear longitudinal slip ratio (-)

Fx_f = Dx_f*sin(cx*atan(bx*sx_f-ex*(bx*sx_f-atan(bx*sx_f)))) - F_roll_f; % Front longitudinal tyre-road force (N)
Fx_r = Dx_r*sin(cx*atan(bx*sx_r-ex*(bx*sx_r-atan(bx*sx_r)))) - F_roll_r; % Rear longitudinal tyre-road force (N)

%% Front and rear lateral tyre-road forces
Dy_f = mu_f*Fz_f;                                               % Pacejka's variable friction-dependent force term - front wheel (N)
Dy_r = mu_r*Fz_r;                                               % Pacejka's variable friction-dependent force term - rear wheel (N)

Fy_f = Dy_f*sin(cy*atan(by*lambda_f-ey*(by*lambda_f-atan(by*lambda_f))));   % Front lateral tyre-road force (N)
Fy_r = Dy_r*sin(cy*atan(by*lambda_r-ey*(by*lambda_r-atan(by*lambda_r))));   % Rear lateral tyre-road force (N)

end

