function [c, ceq] = vehicle_nonlinear_constraints_MS(z0,z_vec,u_vec,th,circuit)
%% Read parameters, states and inputs
% Parameters
m       =       th(1,1);     % vehicle mass (kg)
a       =       th(2,1);     % distance between center of gravity and front axle (m)
b       =       th(3,1);     % distance between center of gravity and rear axle (m)
h       =       th(4,1);     % distance between center of gravity and road (m)
wv      =       th(5,1);     % vehicle width (m)
Izz     =       th(6,1);     % vehicle moment of inertia around z axis(kg*m^2)
Ixz     =       th(7,1);     % vehicle xz product of inertia (kg*m^2)
mux_f   =       th(8,1);     % front road friction coefficient
mux_r   =       th(9,1);     % rear road friction coefficient
rw      =       th(10,1);    % braking front ratio on the front axle [-]
Kr      =       th(11,1);    % rear adimensional cornering stiffness [-]
Kf      =       th(12,1);    % rear adimensional cornering stiffness [-]
rho_air =       th(13,1);    % air density (kg/m^3)
A_f     =       th(14,1);    % vehicle front area (m^2)
Cd      =       th(15,1);    % front aerodynamic drag coefficient

% Ridimensiona z_vec e u_vec nelle matrici originali
N_points = length(circuit(1,:));                % Number of circuit points 
n_states = 7;
n_inputs = 3;
z = reshape(z_vec, n_states, N_points);
u = reshape(u_vec, n_inputs, N_points);

% Inputs
omega_delta      =       u(1,:);     % Steering rate (rad/s)
T_dr             =       u(2,:);     % rear braking torque (Nm)
T_df             =       u(3,:);     % front braking torque (Nm)

%% Compute the state values
% States
N_points = length(circuit(1,:));                % Number of circuit points 
rho     = circuit(1,:);                         % Circuit curvature (1/m) 
w_track = circuit(2,:);                         % Circuit width (m)
w_track_min = min(w_track);                     % Circuit minimum width (m)
delta_s = circuit(9,:);                         % Circuit discretization step (m)
z_dyn_eq(:,1) = z0(:,1);                                    % Initialize the state vector

for s=1:N_points-1
    [z_prime, ~] = vehicle(z_dyn_eq(:,s),u(:,s),th,rho(s));
    z_dyn_eq(:,s+1)     = z_dyn_eq(:,s) + delta_s(s)*z_prime;              % With FFD (we will need to change this)  
end

% States from the dynamic equation
t_dyn_eq        =       z_dyn_eq(1,:);     % curvilinear absissa (m)
vx_dyn_eq       =       z_dyn_eq(2,:);     % body x velocity (m/s)
vy_dyn_eq       =       z_dyn_eq(3,:);     % body y velocity (m/s)
omega_dyn_eq    =       z_dyn_eq(4,:);     % yaw rate (rad/s)
n_dyn_eq        =       z_dyn_eq(5,:);     % transversal displacement (m)
alpha_dyn_eq    =       z_dyn_eq(6,:);     % heading angle (rad)       
delta_dyn_eq    =       z_dyn_eq(7,:);     % steering angle (rad)

% States
t        =       z(1,:);     % time (s)
vx       =       z(2,:);     % body x velocity (m/s)
vy       =       z(3,:);     % body y velocity (m/s)
omega    =       z(4,:);     % yaw rate (rad/s)
n        =       z(5,:);     % transversal displacement (m)
alpha    =       z(6,:);     % heading angle (rad)       
delta    =       z(7,:);     % steering angle (rad)

%% Compute lateral and longitudinal tyre forces
g       =       9.81;                                               % gravity acceleration (m/s^2)
Sr      =       T_dr/rw;                                            % Longitudinal driving/braking force on the front (N)
Sf      =       T_df/rw;                                            % Longitudinal driving/braking force on the front (N)
Nf      =       m*g*b/(a+b)-h/(a+b)*(Sr+Sf);                        % vertical force on front axle (N)
Nr      =       m*g*a/(a+b)+h/(a+b)*(Sr+Sf);                        % vertical force on rear axle (N)
lambda_f=       delta-(a*omega+vy)/vx;                              % front sideslip angle (rad)
lambda_r=       delta-(b*omega-vy)/vx;                              % rear sideslip angle (rad)
Ff      =       Kf*lambda_f.*Nf;                                     % front lateral force (N) 
Fr      =       Kr*lambda_r.*Nr;                                     % front lateral force (N) 
Fxd     =       0.5*rho_air*Cd*A_f*vx.^2;                            % front aerodynamic resistance (N)

%% Constraints definition

% Nonlinear Inequality Constraints
c = [];


% Nonlinear Equality Constraints
ceq = [t - t_dyn_eq;
       vx - vx_dyn_eq;
       vy - vy_dyn_eq;
       omega - omega_dyn_eq;
       n - n_dyn_eq;
       alpha - alpha_dyn_eq;
       delta - delta_dyn_eq];
end