clc;    clear all;      close all;

%% Data from the paper [Dynamics exploration of a single-track rigid car model with load transfer - Rucco et al]

m       = 1480;                                           % vehicle mass (kg)
a       = 1.421;                                          % distance between center of gravity and front axle (m)
b       = 1.029;                                          % distance between center of gravity and rear axle (m)
%Lv      = a+b;                                            % vehicle length (m)
h       = 0.42;                                           % distance between center of gravity and road (m)
wv      = 2;                                              % vehicle width (m)
Izz     = 1950;                                           % vehicle moment of inertia around z axis(kg*m^2)
Ixz     = -50;                                            % vehicle xz product of inertia (kg*m^2)
mux_f   = 1.381;                                          % front road friction coefficient
mux_r   = 1.355;                                          % rear road friction coefficient
rb      = 0.6;                                            % braking front ratio on the front axle 
Kr      = 20;                                             % rear adimensional cornering stiffness 
Kf      = 20;                                             % front adimensional cornering stiffness
rho_air = 1.293;                                          % air density (kg/m^3)
A_f     = 1.9;                                            % vehicle front area (m^2)
Cd      = 0.35;                                           % front aerodynamic drag coefficient 
Voc     = 250;                                            % open circuit battery voltage (V)
Rb      = 0.2;                                            % battery internal resistance (ohm)
eta_r   = 0.96;                                           % rectifier efficiency 
eta_dc  = 0.96;                                           % DC/DC step up converter efficiency
eta_i   = 0.96;                                           % DC/AC inverter efficiency
eta_t   = 0.85;                                           % mechanical transmission efficiency
eta_g   = 0.9;                                            % PMS Generator mean efficiency
eta_m   = 0.9;                                            % PMS Motor mean efficiency
                                           
th = [m;a;b;h;wv;Izz;Ixz;mux_f;mux_r;rb;Kr;Kf;rho_air;A_f;Cd;Voc;Rb;eta_r;eta_dc;eta_i;eta_t;eta_g;eta_m];          

%% Load a circuit

% % Load the Yas Marina Circuit
% load('circuit_points_optimization.mat');

% Load the oval circuit
load("oval_for_optimization.mat");
[rho,w_track,delta_theta,delta_s] = static_variable_computation(xin,yin,xout,yout,xmid,ymid);
circuit = [rho'; w_track'; xin'; yin'; xout'; yout'; xmid'; ymid'; delta_s'];
N_points = length(xmid);
w_track_min = min(w_track);


% "circuit" variable has 9 rows:
% 1st:      rho (1/m) - curvature of every point in the circuit 
% 2nd:      w_track (m) - width of any point in the circuit
% 3rd:      xin (m) - x coordinate of each point of the inner line 
% 4th:      yin (m) - y coordinate of each point of the inner line 
% 5th:      xout (m) - x coordinate of each point of the outer line 
% 6th:      yout (m) - y coordinate of each point of the outer line 
% 7th:      xmid (m) - x coordinate of each point of the center line 
% 8th:      ymid (m) - y coordinate of each point of the center line 
% 9th:      delta_s (m) - distance between each couple of point of the
                          % center line

%% Optimization: Initial state
t0      =       0;          % initial time (s)
Qb0     =       10;         % Battery initial state of charge (Ah)
vx0     =       1;          % body initial x velocity (m/s)
vy0     =       0;          % body initial y velocity (m/s)
omega0  =       0;          % initial yaw rate (rad/s)
s0      =       0;          % initial curvilinear absissa (m)
n0      =       0;          % initial transversal displacement (m)
alpha0  =       0;          % initial heading angle (rad)
delta0  =       0;          % initial steering angle (rad)
z0      =       [t0;Qb0;vx0;vy0;omega0;n0;alpha0;delta0].*ones(1,N_points);
n_states = length([t0;Qb0;vx0;vy0;omega0;n0;alpha0;delta0]);      % Number of states 

%% Definition of the lower and upper bounds on the optimization variables (the input)
% Some of these data are taken by ACAV - lecture 2

omega_delta_min     =   -20*pi/180;                         % Minimum steering rate (rad/s)
Pg_min              =   -eps;                                  % Minimum PMSG power (W)
Pb_min              =   -1e3;                               % Minimum battery power (W)
Ph_min              =   -1e3;                             % Maximum braking power (W)
lb_u                  = [omega_delta_min; Pg_min; Pb_min; Ph_min].*ones(1,N_points);

omega_delta_max     =   20*pi/180;                          % Maximum steering rate (rad/s)
Pe_max              =   1e3;                                % Maximum engine power (W)
Pb_max              =   +1e3;                               % Minimum battery power (W)
Ph_max              =   -eps;                                  % Maximum braking power (W)
ub_u                  = [omega_delta_max; Pe_max*eta_g; Pb_max; Ph_max].*ones(1,N_points);

%% Initial guess for the optimization variables
% u0(1,:)  =   zeros(1,N_points);                              % Initial guess for the steering rate (rad/s)
% u0(2,:)  =   70*ones(1,N_points);                           % Initial guess for the PMSG power (W)
% u0(3,:)  =   70*ones(1,N_points);                           % Initial guess for the battery power (W)
% u0(4,:)  =   -70*ones(1,N_points);                          % Initial guess for the braking power (W)
% n_inputs =   size(u0,1);                                     % Number of inputs

%% Change the initial guess for the optimization variables
u0 = (lb_u+ub_u)/2;
n_inputs =   size(u0,1);                                     % Number of inputs

%% Lower and upper bounds on the states
% ib_max              =   +inf;                            % Maximum battery current (A) 
% Qb_max              =   +inf;                            % Maximum battery SOC (Ah)
% max_values          =   [ib_max; Qb_max];
% 
% % Minimum values
% ib_min           =       -inf;                           % Minimum battery current (A)
% Qb_min           =       -inf;                           % Minimum battery SOC (Ah)
% min_values          =   [ib_min; Qb_min];

%% Launch the optimization routine
% options = optimoptions('fmincon', 'Display', 'iter','MaxFunctionEvaluations',1e5);
% [x_opt, fval] = fmincon(@(z,u) vehicle_cost_function_optimal_control(z,u,th,circuit), u0, [], [], [], [], lb, ub, @(z,u) vehicle_nonlinear_constraints(z0,z,u,th,circuit,min_values,max_values), options);

%% Concatena z e u in un unico vettore di ottimizzazione

% Lower bound on the states
t_min = -eps;                              % Minimum time (s)
Qb_min = -eps;                             % Minimum battery SOC [Ah]
vx_min = -200/3.6;                      % Minimum body x velocity (m/s)
vy_min = -200/3.6;                      % Minimum body y velocity (m/s)
omega_min = -1e2;                       % Minimum yaw rate (rad/s)
n_min = -w_track_min/2;                 % Minimum transversal displacement (m)
alpha_min = -pi/2;                      % Minimum heading angle (rad)                
delta_min = -pi/2;                      % Minimum steering angle (rad)
lb_z = [t_min; Qb_min; vx_min; vy_min; omega_min; n_min; alpha_min; delta_min].*ones(1,N_points);

% Upper bound on the states
t_max = 1e10;                           % Maximum time (s)
Qb_max = 5e3;                           % Maximum battery SOC [Ah]
vx_max = 200/3.6;                      % Maximum body x velocity (m/s)
vy_max = 200/3.6;                      % Maximum body y velocity (m/s)
omega_max = 1e2;                       % Maximum yaw rate (rad/s)
n_max = w_track_min/2;                 % Maximum transversal displacement (m)
alpha_max = pi/2;                      % Maximum heading angle (rad)                
delta_max = pi/2;                      % Maximum steering angle (rad)
ub_z = [t_max; Qb_max; vx_max; vy_max; omega_max; n_max; alpha_max; delta_max].*ones(1,N_points);
% 
% lb = [lb_z; lb_u]; % Concatena i limiti per z e u
% ub = [ub_z; ub_u]; % Concatena i limiti per z e u
% 
% % Definisci la funzione obiettivo e la funzione di vincolo come funzioni di x
% objective_function = @(x) x(1,:)*x(1,:)';
% constraint_function = @(x) vehicle_nonlinear_constraints_MS(z0, x(1:n_states,1:N_points), x(n_states+1:end,1:N_points), th, circuit);
% 
% % Chiama fmincon
% options = optimoptions('fmincon', 'Display', 'iter','MaxFunctionEvaluations',1e5);
% [x_opt, fval] = fmincon(objective_function, x0, [], [], [], [], lb, ub, constraint_function, options);

%% 
% Ridimensiona z0 e u0 in vettori colonna
z0_vec = reshape(z0, [], 1);
u0_vec = reshape(u0, [], 1);

% Concatena z0_vec e u0_vec in un unico vettore di ottimizzazione
x0 = [z0_vec; u0_vec];

% Ridimensiona lb_z, ub_z, lb_u, ub_u in vettori colonna
lb_z_vec = reshape(lb_z, [], 1);
ub_z_vec = reshape(ub_z, [], 1);
lb_u_vec = reshape(lb_u, [], 1);
ub_u_vec = reshape(ub_u, [], 1);

% Concatena lb_z_vec, ub_z_vec, lb_u_vec, ub_u_vec
lb = [lb_z_vec; lb_u_vec];
ub = [ub_z_vec; ub_u_vec];

% Definisci la funzione obiettivo e la funzione di vincolo come funzioni di x
objective_function = @(x) x(1:8:n_states*N_points,:)'*x(1:8:n_states*N_points,:);
constraint_function = @(x) vehicle_nonlinear_constraints_MS(z0, reshape(x(1:n_states*N_points), n_states, N_points), reshape(x(n_states*N_points+1:end), n_inputs, N_points), th, circuit);

% Chiama fmincon
options = optimoptions('fmincon', 'Display', 'iter','MaxFunctionEvaluations',1e5,"Algorithm","interior-point",...
    "EnableFeasibilityMode",true,"BarrierParamUpdate","predictor-corrector",'StepTolerance',eps);
[x_opt, fval] = fmincon(objective_function, x0, [], [], [], [], lb, ub, constraint_function, options);

%% Find the equilibrium positions
t_bar = 0.1;
Qb_bar = 100;
vx_bar = 1;
vy_bar = 0.5;
omega_bar = 0;
n_bar = 0;
alpha_bar = 0;
delta_bar = 0;

z_bar = [t_bar; Qb_bar; vx_bar; vy_bar; omega_bar; n_bar; alpha_bar; delta_bar];

u_bar = equilibrium_position_from_states(z_bar,th,rho);

%% Find the equilibrium positions from inputs
omega_delta_bar = 0;
Pg_bar = 0;
Pb_bar = 10;
Ph_bar = 0;

u_bar2 = [omega_delta_bar; Pg_bar; Pb_bar; Ph_bar];

definisci equilibrium_position_from_inputs