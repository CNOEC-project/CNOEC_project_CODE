clc;    clear all;      close all;

addpath("Functions\");
%% Load the constant parameters
run("Parameters initializations\a0_vehicle_parameters");           % Load the vehicle parameters
run("Parameters initializations\a1_tyre_parameters");              % Load the tyre parameters
% track = load("Circuits\Barcelona_circuit");                      % The track is sampled every 1 meter
track = load("Circuits\YasMarina_circuit2");                       % The track is sampled every 1 meter

% Plot the circuit mid-line
figure;
plot(track.x,track.y,'LineWidth',2);
title('Circuit Mid-line','FontSize',16);
xlabel('$X [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$Y [m]$','Interpreter','LaTex','FontSize',16);
legend('Mid-line','FontSize',14);
grid on;
set(gca, 'FontSize', 16);

% Initialize the collocation parameters
run("Parameters initializations\a2_collocation.m");

%% Optimization: lower and upper bounds on the inputs

n_inputs  =   3;                  % Number of inputs

T_drive_max = 3125;           % Maximum driving torque (Nm)
T_brake_max = eps;                % Maximum braking torque (Nm)
delta_max   = pi/4;               % Maximum steering angle (rad)

T_drive_min = -eps;               % Maximum driving torque (Nm)
T_brake_min = -12e3;              % Maximum braking torque (Nm)
delta_min   = -pi/4;              % Maximum steering angle (rad)

% Define the normalization factor for the input 
u_norm_factor = [max(abs([T_drive_max,T_drive_min]));max(abs([T_brake_max,T_brake_min]));max(abs([delta_max,delta_min]))];

% Normalize the upper bound on the input
ub_u_normalized = [T_drive_max; T_brake_max; delta_max]./u_norm_factor;     
ub_u            = ub_u_normalized.*ones(1,N+1);

% Normalize the lower bound on the input
lb_u_normalized = [T_drive_min; T_brake_min; delta_min]./u_norm_factor; 
lb_u            = lb_u_normalized.*ones(1,N+1);

%% Optimization: lower and upper bounds on the states
n_states      =   7;                            % Number of states

% Lower bound on the states
vx_min = 1e-2;                                  % Minimum body x velocity (m/s)
vy_min = -10;                                   % Minimum body y velocity (m/s)
omega_z_min = -pi/2;                            % Minimum yaw rate (rad/s)
n_min = -6;                                     % Minimum transversal displacement (m)
epsi_min = -pi/4;                               % Minimum heading angle (rad)
omega_f_min = 0;                                % Minimum front wheel rotational speed (rad/s)
omega_r_min = 0;                                % Minimum rear wheel rotational speed (rad/s)

% Upper bound on the states
vx_max = 100;                                   % Maximum body x velocity (m/s)
vy_max = 10;                                    % Maximum body y velocity (m/s)
omega_z_max = pi/2;                             % Maximum yaw rate (rad/s)
n_max = 6;                                      % Maximum transversal displacement (m)
epsi_max = pi/4;                                % Maximum heading angle (rad)
omega_f_max = vx_max/tyre.rw;                   % Maximum front wheel rotational speed (rad/s)
omega_r_max = vx_max/tyre.rw;                   % Maximum rear wheel rotational speed (rad/s)

% Define the normalization factor for the states
z_norm_factor = [max(abs([vx_min,vx_max]));max(abs([vy_min,vy_max]));max(abs([omega_z_min,omega_z_max]));max(abs([n_min,n_max]));max(abs([epsi_min,epsi_max]));max(abs([omega_f_min,omega_f_max]));max(abs([omega_r_min,omega_r_max]))];

% Normalize the upper bound on the states
lb_z_normalized = [vx_min; vy_min; omega_z_min; n_min; epsi_min; omega_f_min; omega_r_min]./z_norm_factor;
lb_z = lb_z_normalized.*ones(1,N+1);
lb_z(1:n_states,1) = -1e10*ones(n_states,1);     % At the initial circuit point, vehicle is still. This is imposed by mean of equality constraints
lb_z(4,N+1) = -0.5/z_norm_factor(4);             % Ask that at the end point the vehicle is close to the center line

% Normalize the lower bound on the states
ub_z_normalized = [vx_max; vy_max; omega_z_max; n_max; epsi_max; omega_f_max; omega_r_max]./z_norm_factor;
ub_z = ub_z_normalized.*ones(1,N+1);
ub_z(1:n_states,1) = 1e10*ones(n_states,1);     % At the initial circuit point, vehicle is still. This is imposed by mean of equality constraints
ub_z(4,N+1) = 0.5/z_norm_factor(4);             % Ask that at the end point the vehicle is close to the center line

%% Optimization: Initial guess for the states

vx0         =       25;             % Initial guess - body x velocity (m/s)
vy0         =       0;             % Initial guess - body y velocity (m/s)
omega_z0    =       0;             % Initial guess - yaw rate (rad/s)
n0          =       0;             % Initial guess - transversal displacement (m)
epsi0       =       0;             % Initial guess - heading angle (rad)
omega_f0    =       vx0/tyre.rw;   % Initial guess - front wheel rotational speed (rad/s)
omega_r0    =       vx0/tyre.rw;   % Intiail guess - rear wheel rotational speed (rad/s)

% Normalize the initial guess for the states
z0_normalized = [vx0;vy0;omega_z0;n0;epsi0;omega_f0;omega_r0]./z_norm_factor;
z0      =       z0_normalized.*ones(1,N+1);
% z0(:,1) = zeros(n_states,1); z0(1,1) = 0.25;

%% Optimization: Initial guess for the inputs

T_drive0         =  2000;           % Initial guess - driving torque (Nm)
T_brake0         =  -1e2;              % Initial guess - braking torque (Nm)
delta_0          =  1e-3;           % Initial guess - steering angle (rad)

% Normalize the initial guess for the inputs
u0_normalized    = [T_drive0;T_brake0;delta_0]./u_norm_factor;
u0               = u0_normalized.*ones(1,N+1);

%% Collect states and inputs contraints in the collocation points
N_col = OPT_d*N;
xc0 = reshape(kron(z0(:,1:end-1),ones(1,OPT_d)),OPT_d*N*n_states,1); % Constant interpolation between z0

%% Prepare the optimization

% Reshape z0, u0 and xc0 into column vectors
z0_vec = reshape(z0, [], 1);
u0_vec = reshape(u0, [], 1);
xc0_vec= reshape(xc0,[], 1);

% Stack z0_vec, u0_vec and xc0_vec in a unique optimization vector
x0 = [z0_vec; u0_vec; xc0_vec];

% Reshape lb_z, ub_z, lb_u, ub_u, lb_z_normalized, ub_z_normalized into column vectors
lb_z_vec = reshape(lb_z, [], 1);
ub_z_vec = reshape(ub_z, [], 1);
lb_u_vec = reshape(lb_u, [], 1);
ub_u_vec = reshape(ub_u, [], 1);
lb_xc_vec= repmat(lb_z_normalized,N*OPT_d,1);
ub_xc_vec= repmat(ub_z_normalized,N*OPT_d,1);

% Stack lb_z_vec, lb_u_vec, lb_xc_vec and ub_z_vec, ub_u_vec, ub_xc_vec 
lb = [lb_z_vec; lb_u_vec; lb_xc_vec];
ub = [ub_z_vec; ub_u_vec; ub_xc_vec];


nz_grid_points = length(z0_vec);        % Store the total number of states in the grid points 
nu_grid_points = length(u0_vec);        % Store the total number of inputs in the grid points 
nz_col_points  = length(xc0_vec);       % Store the total number of states in the collocation points 

%% Set the linear equality matrices
% % Impose null states at starting point (i.e., z(s=1)==0)
Aeq = zeros(n_states,length(x0));
Aeq(:,1:n_states) = eye(n_states);
beq = zeros(n_states,1);
beq(1) = 1e-2;    % To help the solver, ask a not exactly null initial velocity, to avoid tprime = inf 

%% Set the linear inequality matrices (based on lb and ub)
C_ineq = [eye(length(x0)); -eye(length(x0))];
d_ineq = [lb; -ub];

%% Launch the optimization with myfmincon

% Set the number of nonlinear constraints
p = (N+1) + OPT_d*n_states*N + n_states*N;              % Number of equality constraints
% p = OPT_d*n_states*N + n_states*N;              % Number of equality constraints
q = (N+1)*2;                                            % Number of inequality constraints - friction ellipse at grid points

% Initialize solver options
myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'BFGS';
myoptions.gradmethod  	=	'CD';
myoptions.graddx        =	2^-17;
myoptions.tolconstr     =   10;
myoptions.ls_nitermax   =	5e2;
myoptions.nitermax      =	1e3;
myoptions.GN_funF       =   @(x) objective_function_GN([x(1:nz_grid_points); x(nz_grid_points+nu_grid_points+1:end)],x(length(z0_vec)+1:length(z0_vec)+length(u0_vec)),N,chassis,tyre,s_col,z_norm_factor,u_norm_factor,k_col,B_col,nz_grid_points,OPT_d);
myoptions.tolfun    	=	1e-6;      
myoptions.ls_c          =	0.99;                                  

tic;
% Run solver
[x_opt,f_val,niter,exitflag,xsequence] = myfmincon(@(x) vehicle_cost_and_constraints_BFGS([x(1:nz_grid_points); x(nz_grid_points+nu_grid_points+1:end)],x(length(z0_vec)+1:length(z0_vec)+length(u0_vec)),N,chassis,tyre,s_col,z_norm_factor,u_norm_factor,k_col,C_col,D_col,nz_grid_points,OPT_d,B_col), ...
                                                                                x0,Aeq,beq,C_ineq,d_ineq,p,q,myoptions);
elapsed_time = toc

%% Extract the results

z_grid_opt_vec = x_opt(1:nz_grid_points);                                % Normalized states at the grid points
u_opt_vec      = x_opt(nz_grid_points+1:nz_grid_points+nu_grid_points);  % Normalized inputs at the grid points
z_col_opt_vec  = x_opt(nz_grid_points+nu_grid_points+1:end);             % Normalized inputs at the colocation points

total_lap_time_opt   = sqrt(f_val);                                      % Optimal lap time [s]                    
laptime_grid_vec_opt = optimal_laptime_extraction([z_grid_opt_vec;  z_col_opt_vec],u_opt_vec,N,chassis,tyre,s_col,z_norm_factor,u_norm_factor,k_col,B_col,nz_grid_points,OPT_d);
laptime_vec_opt_full = interp1(s_grid', laptime_grid_vec_opt', s_full, 'linear')';

z_grid_opt = reshape(z_grid_opt_vec,n_states,N+1).*z_norm_factor;        % States at the grid points
u_opt = reshape(u_opt_vec,n_inputs,N+1).*u_norm_factor;                  % Inputs at the grid points
z_col_opt = reshape(z_col_opt_vec,n_states,OPT_d*N).*z_norm_factor;      % States at the collocation points

% Extract the full state vector
% z_full = [Zk Zk,1 Zk,2 Zk,3 Zk+1 Zk+1,1 ...]
z_full = kron(z_grid_opt(:,1:end-1), [1 zeros(1,OPT_d)]) + reshape([zeros(n_states,N); reshape(z_col_opt,n_states*OPT_d,N)],n_states,[]);
z_full(:,end+1) = z_grid_opt(:,end);

% Extract the full input vector
u_full = interp1(s_grid', u_opt', s_full, 'previous')';              % Constant inputs between grid points

% States
vx_full_opt         =       z_full(1,:);                             % Optimal body x velocity (m/s) - at grid points
vy_full_opt         =       z_full(2,:);                             % Optimal - body y velocity (m/s) - at grid points
omega_full_opt    =         z_full(3,:);                             % Optimal - yaw rate (rad/s) - at grid points
n_full_opt          =       z_full(4,:);                             % Optimal - transversal displacement (m) - at grid points
epsi_full_opt       =       z_full(5,:);                             % Optimal - heading angle (rad) - at grid points
omega_f_full_opt    =       z_full(6,:);                             % Optimal - front wheel rotational speed (rad/s) - at grid points
omega_r_full_opt    =       z_full(7,:);                             % Optimal - rear wheel rotational speed (rad/s) - at grid points

% Inputs
T_drive_opt_full         =  u_full(1,:);                             % Optimal driving torque (Nm) - at grid points
T_brake_opt_full         =  u_full(2,:);                             % Optimal braking torque (Nm) - at grid points
delta_opt_full           =  u_full(3,:);                             % Optimal steering angle (rad) - at grid points

%% Postprocessing - Collect additional data

track_new.s = s_full;                                                                    % Curvilinear absissa at grid and collocation points
track_new.k = k_full;                                                                    % Curvature at grid and collocation points

track_new.x = interp1(track.s, track.x, track_new.s);                                    % Mid-line x-coordinate at grid and collocation points   
track_new.y = interp1(track.s, track.y, track_new.s);                                    % Mid-line y-coordinate at grid and collocation points  

[track_new.xopt,track_new.yopt] = cartPath(track_new.x,track_new.y, z_full(4,:));        % Reconstruct the optimal racing line
[track_new.Xl,track_new.Xr] = trackLimits(track_new.x,track_new.y, z_norm_factor(4)*2);  % Reconstruct the track limits

%% Plot the results

% States
figure;
plot(s_full,vx_full_opt*3.6,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,vx_max*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,vx_min*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(1,:)*z_norm_factor(1)*3.6,'LineWidth',2,'Color',[0 1 0]);
legend('$v_{x,opt}$','$v_{x,max}$','$v_{x,min}$','$v_{x,init}$','interpreter','latex','FontSize',16);
title('Longitudinal speed','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$v_{x} \, [\frac{km}{h}]$','Interpreter','LaTex','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,vy_full_opt*3.6,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,vy_max*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,vy_min*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(2,:)*z_norm_factor(2)*3.6,'LineWidth',2,'Color',[0 1 0]);
legend('$v_{y,opt}$','$v_{y,max}$','$v_{y,min}$','$v_{y,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$v_{y} \, [\frac{km}{h}]$','Interpreter','LaTex','FontSize',16);
title('Lateral speed','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,omega_full_opt,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,omega_z_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,omega_z_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(3,:)*z_norm_factor(3),'LineWidth',2,'Color',[0 1 0]);
legend('$\Omega_{z,opt}$','$\Omega_{z,max}$','$\Omega_{z,min}$','$\Omega_{z,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\Omega_{z} \, [\frac{rad}{s}]$','Interpreter','LaTex','FontSize',16);
title('Yaw rate','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,n_full_opt,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,n_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,n_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(4,:)*z_norm_factor(4),'LineWidth',2,'Color',[0 1 0]);
legend('$n_{opt}$','$n_{max}$','$n_{min}$','$n_{init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$n [m]$','Interpreter','LaTex','FontSize',16);
title('Transversal displacement','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,rad2deg(epsi_full_opt),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,rad2deg(epsi_max)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,rad2deg(epsi_max)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,rad2deg(z0(5,:)*z_norm_factor(5)),'LineWidth',2,'Color',[0 1 0]);
legend('$\epsilon_{opt}$','$\epsilon_{max}$','$\epsilon_{min}$','$\epsilon_{init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\epsilon \, [deg]$','Interpreter','LaTex','FontSize',16);
title('Heading angle','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,omega_f_full_opt*60/(2*pi),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,omega_f_max*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,omega_f_min*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(6,:)*z_norm_factor(6)*60/(2*pi),'LineWidth',2,'Color',[0 1 0]);
legend('$\Omega_{f,opt}$','$\Omega_{f,max}$','$\Omega_{f,min}$','$\Omega_{f,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\Omega_{f} \, [rpm]$','Interpreter','LaTex','FontSize',16);
title('Front wheel rotational speed','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,omega_r_full_opt*60/(2*pi),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,omega_r_max*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,omega_r_min*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(7,:)*z_norm_factor(7)*60/(2*pi),'LineWidth',2,'Color',[0 1 0]);
legend('$\Omega_{r,opt}$','$\Omega_{r,max}$','$\Omega_{r,min}$','$\Omega_{r,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\Omega_{r} \, [rpm]$','Interpreter','LaTex','FontSize',16);
title('Rear wheel rotational speed','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

% Inputs
figure;
plot(s_full,T_drive_opt_full,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,T_drive_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,T_drive_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,u0(1,:)*u_norm_factor(1),'LineWidth',2,'Color',[0 1 0]);
legend('$T_{drive,opt}$','$T_{drive,max}$','$T_{drive,min}$','$T_{drive,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$T_{drive} \, [Nm]$','Interpreter','LaTex','FontSize',16);
title('Driving torque','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,T_brake_opt_full,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,T_brake_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,T_brake_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,u0(2,:)*u_norm_factor(2),'LineWidth',2,'Color',[0 1 0]);
legend('$T_{brake,opt}$','$T_{brake,max}$','$T_{brake,min}$','$T_{brake,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$T_{brake} \, [Nm]$','Interpreter','LaTex','FontSize',16);
title('Braking torque','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,rad2deg(delta_opt_full),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,rad2deg(delta_max)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,rad2deg(delta_min)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,rad2deg(u0(3,:)*u_norm_factor(3)),'LineWidth',2,'Color',[0 1 0]);
legend('$\delta_{opt}$','$\delta_{max}$','$\delta_{min}$','$\delta_{init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\delta \, [deg]$','Interpreter','LaTex','FontSize',16)
title('Steering angle','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,T_brake_opt_full,'b','LineWidth',2); hold on;
plot(s_full,T_drive_opt_full,'r','LineWidth',2);
legend('$T_{brake,opt}$','$T_{drive,opt}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$T \, [Nm]$','Interpreter','LaTex','FontSize',16);
title('Braking torque vs Driving Torque','FontSize',16);
grid on;
set(gca, 'FontSize', 16);
%% Plot optimal racing line with the corresponding state values (point-by-point)
% Here you can click on the figure to see the state values at the desired
% point

figure;
plot(track_new.Xl(:,1), track_new.Xl(:,2), 'LineWidth', 1,'Color',[0 0 0]); hold on;
plot(track_new.Xr(:,1), track_new.Xr(:,2), 'LineWidth', 1,'Color',[0.5 0.5 0.5]); hold on;
plot(track_new.xopt, track_new.yopt, 'LineWidth', 1.5,'Color',[0 0.5 0]);
plot(track_new.xopt(1), track_new.yopt(1),'.', 'MarkerSize', 15,'Color',[1 0 0]);
legend('Outer line', 'Inner line', 'Optimal racing line','Starting point', 'FontSize', 14);
title('Optimal solution', 'FontSize', 16);
xlabel('$X [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
ylabel('$Y [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
grid on;
set(gca, 'FontSize', 16);

% Save the plot handle
hFig = gcf;

% Initialize handle for text object in the appdata of the figure
setappdata(hFig, 'hText', []);

% Set the callback function for mouse click event
set(hFig, 'WindowButtonDownFcn', @mouseClickCallback);


%%
Fz_f = zeros(size(s_full));
Fz_r= zeros(size(s_full));
Fx_f= zeros(size(s_full));
Fx_r= zeros(size(s_full));
Fy_f= zeros(size(s_full));
Fy_r= zeros(size(s_full));
mu_f= zeros(size(s_full));
mu_r= zeros(size(s_full));
c1 = zeros(size(s_full));
c2 = zeros(size(s_full));

for ii=1:length(s_full)
    [Fz_f(ii),Fz_r(ii),Fx_f(ii),Fx_r(ii),Fy_f(ii),Fy_r(ii),mu_f(ii),mu_r(ii)] = friction_ellipse_forces(z_full(:,ii),u_full(:,ii),chassis,tyre);
    c1(ii) = mu_f(ii)^2-(Fx_f(ii)^2+Fy_f(ii)^2)./Fz_f(ii)^2;
    c2(ii) = mu_r(ii)^2-(Fx_r(ii)^2+Fy_r(ii)^2)./Fz_r(ii)^2;
end

































return
%% Save the solution
save('Solutions\solution_BFGS4');

%% Create and save a video with the final solution

numFrames = length(s_full);                                      % Adjusted to match loop step
frames_vec(numFrames) = struct('cdata', [], 'colormap', []);     % Initialize an array to store the frames

for ii=1:length(s_full)
    hFig = figure(30);
    plot(track_new.Xl(:,1), track_new.Xl(:,2), 'LineWidth', 1,'Color',[0 0 0]); hold on;
    plot(track_new.Xr(:,1), track_new.Xr(:,2), 'LineWidth', 1,'Color',[0.5 0.5 0.5]); hold on;
    plot(track_new.xopt, track_new.yopt, 'LineWidth', 1.5,'Color',[0 0.5 0]); hold on;
    plot(track_new.xopt(1), track_new.yopt(1),'.', 'MarkerSize', 15,'Color',[1 0 0]); hold on;
    plot(track_new.xopt(ii), track_new.yopt(ii),'^', 'MarkerSize', 7,'Color',[0 0 1],'MarkerFaceColor','b'); hold off;
    legend('Outer line', 'Inner line', 'Optimal racing line','Starting point','Vehicle position', 'FontSize', 14);
    title(['Optimal lap simulation - current time: ' num2str(round(laptime_vec_opt_full(ii)*100)/100), ' s'], 'FontSize', 16);
    xlabel('$X [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
    ylabel('$Y [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
    grid on;
    set(gca, 'FontSize', 16);

    % Display the values of the selected variables at the selected point
    textString = sprintf(['$v_{x}$ = %.2f $\\frac{km}{h}$ \n $v_{y}$ = %.2f $\\frac{km}{h}$ \n $\\Omega_{z}$ = %.2f $\\frac{rad}{s}$ \n $n$ = %.2f $m$ \n $\\epsilon$ = %.2f $^{\\circ}$ \n' ...
                          '$\\Omega_{f}$ = %.2f $rpm$ \n $\\Omega_{r}$ = %.2f $rpm$ \n $T_{drive}$ = %.2f $Nm$ \n $T_{brake}$ = %.2f $Nm$ \n' ...
                          '$\\delta$ = %.2f $^{\\circ}$ \n $s$ = %.2f $m$'], ...
                          vx_full_opt(ii)*3.6, vy_full_opt(ii)*3.6, omega_full_opt(ii), n_full_opt(ii), ...
                          rad2deg(epsi_full_opt(ii)), omega_f_full_opt(ii)*60/(2*pi), omega_r_full_opt(ii)*60/(2*pi), ...
                          T_drive_opt_full(ii), T_brake_opt_full(ii), rad2deg(delta_opt_full(ii)), s_full(k));
   
    hText = text(track_new.xopt(ii)+10,  track_new.yopt(ii)+10, textString, 'FontSize', 12, 'BackgroundColor', 'white', 'Interpreter', 'latex');
    pause(0.01);
    % Capture the frame
    frames_vec(ii) = getframe(hFig);
    clear hText;
end

% Save the video
hFigVisible = figure;
set(hFigVisible, 'Position', [100, 100, 1200, 800]);
set(hFigVisible, 'PaperPositionMode', 'auto');
set(hFigVisible, 'Units', 'normalized');
set(hFigVisible, 'Position', [0, 0, 1, 1]);
set(hFigVisible, 'Visible', 'on');
frameRate = round(numFrames/video_duration);
movie(hFigVisible, frames_vec, 1, frameRate); % Play the movie once at the specified frame rate
video_duration = total_lap_time_opt;

outputVideo = VideoWriter('BFGS_optimal_lap', 'MPEG-4'); % You can specify 'Uncompressed AVI' or other formats if needed
outputVideo.FrameRate = frameRate; % Set the desired frame rate

% Open the video writer object
open(outputVideo);

% Write each frame to the video
for ii = 1:length(frames_vec)
    writeVideo(outputVideo, frames_vec(ii));
end