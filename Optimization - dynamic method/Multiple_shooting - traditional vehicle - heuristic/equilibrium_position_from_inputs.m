function z_bar = equilibrium_position_from_inputs(u_bar,th,rho)
%EQUILIBRIUM_POSITION Computes the equilibrium position z_bar corresponding to the
% constant input u_bar.
% 
% Inputs:
%           u_bar   (chosen constant input)
%           th      (model parameters)
%           rho     (circuit curvature)
% Outputs: 
%           z_bar - equilibrium position corresponding to the constant input

%% Read parameters, states and inputs
% Parameters
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

% States
t            =       @(z) (z(1));     % curvilinear absissa (m)
vx           =       @(z) (z(2));     % body x velocity (m/s)
vy           =       @(z) (z(3));     % body y velocity (m/s)
omega        =       @(z) (z(4));     % yaw rate (rad/s)
n            =       @(z) (z(5));     % transversal displacement (m)
alpha        =       @(z) (z(6));     % heading angle (rad)       
delta        =       @(z) (z(7));     % steering angle (rad)

n_states     =       7;               % number of states

% Inputs
omega_delta      =       u_bar(1,1);     % Steering rate (rad/s)
T_dr             =       u_bar(2,1);     % rear braking torque (Nm)
T_df             =       u_bar(3,1);     % front braking torque (Nm)

n_inputs         =       length(u_bar);  % number of inputs   

%% Compute lateral and longitudinal tyre forces
g       =       9.81;                                               % gravity acceleration (m/s^2)
Sr      =       T_dr/rw;                                            % Longitudinal driving/braking force on the front (N)
Sf      =       T_df/rw;                                            % Longitudinal driving/braking force on the front (N)
Nf      =       m*g*b/(a+b)-h/(a+b)*(Sr+Sf);                        % vertical force on front axle (N)
Nr      =       m*g*a/(a+b)+h/(a+b)*(Sr+Sf);                        % vertical force on rear axle (N)
lambda_f=       @(z) delta(z)-(a*omega(z)+vy(z))/vx(z);                              % front sideslip angle (rad)
lambda_r=       @(z) delta(z)-(b*omega(z)-vy(z))/vx(z);                              % rear sideslip angle (rad)
Ff      =       @(z) Kf*lambda_f(z)*Nf;                                     % front lateral force (N) 
Fr      =       @(z) Kr*lambda_r(z)*Nr;                                     % front lateral force (N) 
Fxd     =       @(z) 0.5*rho_air*Cd*A_f*vx(z)^2;                            % front aerodynamic resistance (N)


% Model equations  
fun  =  @(z)  [(1-n(z)*rho)/(vx(z)*cos(alpha(z))-vy(z)*sin(alpha(z)));            % t_bar
                         ((Sr+Sf*cos(delta(z))-Ff(z)*sin(delta(z))-Fxd(z))/m+omega(z)*vy(z));      % vx_bar
                        ((Fr(z)+Sf*sin(delta(z))+Ff(z)*cos(delta(z)))/m-omega(z)*vx(z));           % vy_bar
                        ((a*Ff(z)*cos(delta(z))+a*Sf*sin(delta(z))-b*Fr(z))/Izz);                      % omega_bar
                        (vx(z)*cos(alpha(z))+vy(z)*sin(alpha(z)));                           % n_bar
                        (omega(z)-(vx(z)*cos(alpha(z))-vy(z)*sin(alpha(z)))/(1-n(z)*rho)*rho);     % alpha_bar
                        omega_delta;                                                            % delta_bar
                        ];

z0 = ones(n_states,1);                                                    % Initial guess for the solution
options = optimoptions('fsolve', 'MaxFunctionEvaluations', 1e5);
z_bar = fsolve(fun, z0, options);


end

