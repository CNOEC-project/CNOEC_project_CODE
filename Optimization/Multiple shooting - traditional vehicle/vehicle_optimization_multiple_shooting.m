clc;    clear all;      close all;

%% Data from the paper [Dynamics exploration of a single-track rigid car model with load transfer - Rucco et al]

m       = 1480;                                           % vehicle mass (kg)
a       = 1.421;                                          % distance between center of gravity and front axle (m)
b       = 1.029;                                          % distance between center of gravity and rear axle (m)
%Lv      = a+b;                                            % vehicle length (m)
h       = 0.42;                                           % distance between center of gravity and road (m)
wv      = 0.5;                                              % vehicle width (m)
Izz     = 1950;                                           % vehicle moment of inertia around z axis(kg*m^2)
Ixz     = -50;                                            % vehicle xz product of inertia (kg*m^2)
mux_f   = 1.381;                                          % front road friction coefficient
mux_r   = 1.355;                                          % wheel radius (m)
rw      = 0.3;                                            % braking front ratio on the front axle 
Kr      = 20;                                             % rear adimensional cornering stiffness 
Kf      = 20;                                             % front adimensional cornering stiffness
rho_air = 1.293;                                          % air density (kg/m^3)
A_f     = 1.9;                                            % vehicle front area (m^2)
Cd      = 0.35;                                           % front aerodynamic drag coefficient 
                                           
th = [m;a;b;h;wv;Izz;Ixz;mux_f;mux_r;rw;Kr;Kf;rho_air;A_f;Cd];          

%% Load a circuit

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

%% Optimization: lower and upper bounds on the inputs
% Some of these data are taken by ACAV - lecture 2, others are from lab 1
% of CNOEC

omega_delta_min     =   -20*pi/180;                         % Minimum steering rate (rad/s)
T_dr_min            =   -5100;                              % Minimum rear braking torque (Nm)
T_df_min            =   -5100;                              % Minimum front braking torque (Nm)

lb_u                  = [omega_delta_min; T_dr_min; T_df_min].*ones(1,N_points);


omega_delta_max     =   20*pi/180;                          % Maximum steering rate (rad/s)
T_dr_max            =   890;                                % Minimum rear braking torque (Nm)
T_df_max            =   890;                                % Minimum front braking torque (Nm)

ub_u                  = [omega_delta_max; T_dr_max; T_df_max].*ones(1,N_points);

%% Optimization: lower and upper bounds on the states
% Some of these data are taken by ACAV - lecture 2, others are from lab 1
% of CNOEC

% Lower bound on the states
t_min = 0;                              % Minimum time (s)
vx_min = -200/3.6;                      % Minimum body x velocity (m/s)
vy_min = -200/3.6;                      % Minimum body y velocity (m/s)
omega_min = -1e2;                       % Minimum yaw rate (rad/s)
n_min = -w_track_min/2;                 % Minimum transversal displacement (m)
alpha_min = -pi/2;                      % Minimum heading angle (rad)                
delta_min = -pi/2;                      % Minimum steering angle (rad)
lb_z = [t_min; vx_min; vy_min; omega_min; n_min; alpha_min; delta_min].*ones(1,N_points);

% Upper bound on the states
t_max = inf;                           % Maximum time (s)
vx_max = 200/3.6;                      % Maximum body x velocity (m/s)
vy_max = 200/3.6;                      % Maximum body y velocity (m/s)
omega_max = 1e2;                       % Maximum yaw rate (rad/s)
n_max = w_track_min/2;                 % Maximum transversal displacement (m)
alpha_max = pi/2;                      % Maximum heading angle (rad)                
delta_max = pi/2;                      % Maximum steering angle (rad)
ub_z = [t_max; vx_max; vy_max; omega_max; n_max; alpha_max; delta_max].*ones(1,N_points);

n_states = size(lb_z,1);               % Number of states

%% Optimization: Initial guess for the optimization variables (the input)
% 
% omega_delta0 = 0;                              % Initial guess for the steering rate (rad/s)
% T_dr0 = 0;                                     % Initial guess for the rear traction torque (Nm)
% T_df0 = 0;                                     % Initial guess for the front traction torque (Nm)
% 
% u0 = [omega_delta0; T_dr0; T_df0].*ones(1,N_points);
% n_inputs =   length([omega_delta0; T_dr0; T_df0]);             % Number of inputs

%% Optimization: Initial guess for the optimization variables (the states)
% 
% t0      =       eps;        % initial time (s)
% vx0     =       1;          % body initial x velocity (m/s)
% vy0     =       0;          % body initial y velocity (m/s)
% omega0  =       0;          % initial yaw rate (rad/s)
% n0      =       0;          % initial transversal displacement (m)
% alpha0  =       0;          % initial heading angle (rad)
% delta0  =       0;          % initial steering angle (rad)
% 
% z0      =       [t0;vx0;vy0;omega0;n0;alpha0;delta0].*ones(1,N_points);
% 
% n_states = length([t0;vx0;vy0;omega0;n0;alpha0;delta0]);      % Number of states 

%% Optimization: Initial guess for the optimization variables (the input)

u0(1,:) = linspace(0,1e-2,N_points);
u0(2,:) = linspace(1,10,N_points);
u0(3,:) = linspace(1,10,N_points);
n_inputs =   size(u0,1);             % Number of inputs

% Corresponding initial guess for the states
z0 = zeros(n_states,N_points);
for ii=1:N_points
    z0(:,ii) = equilibrium_position_from_inputs(u0(:,ii),th,rho);
end

%% Prepare and launch the optimization

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
objective_function = @(x) x(1:n_states:n_states*N_points,:)'*x(1:n_states:n_states*N_points,:);
constraint_function = @(x) vehicle_nonlinear_constraints_MS(z0, reshape(x(1:n_states*N_points), n_states, N_points), reshape(x(n_states*N_points+1:end), n_inputs, N_points), th, circuit);

% Chiama fmincon
options = optimoptions('fmincon', 'Display', 'iter','MaxFunctionEvaluations',1e5);
[x_opt, fval] = fmincon(objective_function, x0, [], [], [], [], lb, ub, constraint_function, options);

%% Extract the results
t_opt     = x_opt(1:n_states:n_states*N_points);              % Optimal time vector (s)
vx_opt    = x_opt(2:n_states:n_states*N_points);              % Optimal longitudinal velocity (m/s)
vy_opt    = x_opt(3:n_states:n_states*N_points);              % Optimal lateral velocity (m/s)
omega_opt = x_opt(4:n_states:n_states*N_points);              % Optimal yaw rate (rad/s)
n_opt     = x_opt(5:n_states:n_states*N_points);              % Optimal transversal displacement (m)
alpha_opt = x_opt(6:n_states:n_states*N_points);              % Optimal heading (rad)
delta_opt = x_opt(7:n_states:n_states*N_points);              % Optimal steering angle (rad)

omega_delta_opt = x_opt(n_states*N_points+1:n_inputs:end);    % Optimal steering rate (rad/s)
optimal_T_dr    = x_opt(n_states*N_points+2:n_inputs:end);    % Optimal rear traction torque (Nm)
optimal_T_df    = x_opt(n_states*N_points+3:n_inputs:end);    % Optimal front traction torque (Nm)

Tlap_opt = sum(t_opt);                                        % Optimal lap time (s)

%% Plot the results
s_circuit = cumsum(delta_s);                                  % Circuit mid line curvilinear absissa (m)

figure;
plot(s_circuit,t_opt); hold on;
plot(s_circuit,z0(1,:)); 
legend('Optimal','Initial guess');
title('delta T optimal');

figure;
plot(s_circuit,vx_opt); hold on;
plot(s_circuit,z0(2,:));
legend('Optimal','Initial guess');
title('vx optimal');

figure;
plot(s_circuit,vy_opt); hold on;
plot(s_circuit,z0(3,:));
legend('Optimal','Initial guess');
title('vy optimal');

figure;
plot(s_circuit,omega_opt); hold on;
plot(s_circuit,z0(4,:));
legend('Optimal','Initial guess');
title('omega optimal');

figure;
plot(s_circuit,n_opt); hold on;
plot(s_circuit,z0(5,:));
legend('Optimal','Initial guess');
title('n optimal');

figure;
plot(s_circuit,alpha_opt); hold on;
plot(s_circuit,z0(6,:)); 
legend('Optimal','Initial guess');
title('alpha optimal');

figure;
plot(s_circuit,delta_opt); hold on;
plot(s_circuit,z0(7,:));
legend('Optimal','Initial guess');
title('delta optimal');

%% Extract the optimal racing line
x_opt_rl = zeros(size(xmid));
y_opt_rl = zeros(size(xmid));

for ii=1:N_points
    [x_opt_rl(ii), y_opt_rl(ii)] = extract_optimal_rl(xin(ii), yin(ii), xout(ii), yout(ii), xmid(ii), ymid(ii), n_opt(ii));    
end

figure;
plot(xin,yin,'*','Color',[0 0 0]); hold on;
plot(xout,yout,'*','Color',[0.5 0.5 0.5]); hold on;
plot(xmid,ymid,'*','Color',[0 0 1]); hold on;
plot(x_opt_rl,y_opt_rl,'Color',[0 1 0]);