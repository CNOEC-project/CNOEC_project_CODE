% Minimum lap time problem solver
%
% Solve the Minimum Lap Time Problem, i.e. find the 
% optimal race line and control inputs to minimise the lap time.

%% Initialisation
% Clear Simulink SDI callback if it exists
global figures
try Simulink.sdi.unregisterCursorCallback(figures.callbackID); end 

% Clear workspace
% clc
clear; clear global;

% Import functions
addpath('Functions'); %Include useful functions

elapsedTime = 0; tic %Start time counter to know how long takes to optimize a lap time

%% Car and track models

% Vehicle model
run('Parameters\vehParams.m'); %load vehicle parameters

% Track model
%-A) Self defined virtual tracks
%%--Define curvature
track.k = [zeros(1,150) pi/25*ones(1,20) zeros(1,50) -pi/125*ones(1,90) -pi/50*ones(1,35) pi/50*ones(1,35) -pi/500*(1:0.05:6) zeros(1,200) -pi/20*ones(1,20) zeros(1,80)]; %Virtual track
%%-Smooth curvature and create distance array
track.k = simpleMA(track.k,10,2); % Smoothen curvature signal
track.s = linspace(0,2*length(track.k),length(track.k));% (assume each element of the curvature array is spaced by 1m)

%-B) Real circuits data 
% track = load('Parameters/Barcelona_circuit.mat');
% track = load('Parameters/YasMarina_circuit.mat');

%% User options

% Optimal Control Problem - Boundary constraints
%-initial   ( x = [vx vy r n epsi Om_f Om_r] )
vi = 25;
Xi = [vi; nan; nan; nan; nan; nan; nan];
%-final
vf = nan;
Xf = [vf; nan; nan; nan; nan; nan; nan];

% Collocation
OPT_ds = 3; %(m) collocation step
OPT_d = 3; % degree of interpolating polynomials
OPT_uinter = 'linear'; % assume 'linear' or 'constant' inputs
OPT_e = 1*1e-3; %Small quantity. Used as the slack for the path constraints and wherever helpful (like setting the initial guesses of the solver)

% Solver options
options = optimoptions('fmincon', 'Display', 'iter','MaxFunctionEvaluations',1e4);

%Constraints on the rate of inputs (units of each input per second)
duk_ub = [1e4; 1e4; 1]; %upper bound
duk_lb = [-1e4; -1e4; -1]; %lower bound

% Regularisation factors for the...
rdu = [0.01; 0.01; 0.1]; rdu2 = [0.01; 0.01; 10]; %...first and second derivatives of the inputs
rdy = 0.001; rdy2 = 0.015; %...first and second derivatives of the aux variables
ru = 0.1; %...steering angle

%% Vehicle model
% Define vehicle model scaling factors

%%%%%%% State variables scaling and bounds %%%%%%%%%%%%%%%%%%
nx = 7; % number of state variables

% longitudinal velocity [m/s]
vx_s = 100;
%vx = vx_s * vx_n -> the decision variable it's vx_n

% lateral velocity [m/s]
vy_s = 10;
%vy = vy_s * vy_n -> the decision variable it's vy_n 

% yaw rate [rad/s]
r_s = 1;
% r = r_s * r_n -> the decision variable it's r_n

% lateral distance to centreline [m] - left of centreline => n > 0; right => n < 0
n_s = 5;
% n = n_s * n_n -> the decision variable it's n_n

% angle to centreline tangent direction [rad]
epsi_s = 1;
% epsi = epsi_s * epsi_n -> the decision variable it's epsi_n

% angular velocity front tyre [rad/s]
Om_f_s = vx_s/vp.Rw;
% Om_f = Om_f_s * Om_f_n ->the decision variable it's Om_f_n  

% angular velocity rear tyre [rad/s]
Om_r_s = vx_s/vp.Rw;
% Om_r = Om_r_s * Om_r_n ->the decision variable it's Om_r_n

%%-State limits
vx_lim = 1/vx_s * [OPT_e vp.v_max];
vy_lim = 1/vy_s * [-10 10];
r_lim = 1/r_s * [-pi/2 pi/2];
n_lim = 1/n_s * [-4 4]; % Constant track limits. Variable width tracks can be implemented but in a different way
epsi_lim = 1/epsi_s * [-pi/4 pi/4];
Om_f_lim = 1/Om_f_s * [0 vp.v_max/vp.Rw];
Om_r_lim = 1/Om_r_s * [0 vp.v_max/vp.Rw];

% scaling factors
x_s = [vx_s; vy_s; r_s; n_s; epsi_s; Om_f_s; Om_r_s];       
% states vector decision variable
% x_n = [vx_n; vy_n; r_n; n_n; epsi_n; Om_f_n; Om_r_n];

% limits
x_lim = [vx_lim; vy_lim; r_lim; n_lim; epsi_lim; Om_f_lim; Om_r_lim];
x_min = x_lim(:,1);
x_max = x_lim(:,2);

%Check number of states
if nx ~= length(x_s) 
    error('Number of states is not consistent');
end

%%%%%%% Control variables (inputs) scaling and bounds %%%%%%% 

% number of control variables
nu = 3;

% driving torque (Nm)
Tdrive_s = vp.Tdrive_max;
% Tdrive = Tdrive_s * Tdrive_n;

% braking torque [Nm]
Tbrake_s = vp.Tbrake_max;
% Tbrake = Tbrake_s * Tbrake_n;

% steer angle [rad]
delta_s = pi/8;
% delta = delta_s * delta_n;

%%-Input limits
Tdrive_lim = 1/Tdrive_s * [0 vp.Tdrive_max];
Tbrake_lim = 1/Tbrake_s * [-vp.Tbrake_max 0];
delta_lim = 1/delta_s * [-pi/4 pi/4];

% scaling factors for inputs
u_s = [Tdrive_s; Tbrake_s; delta_s]; 
% inputs vector (scaled) -decision variable
% u = [Tdrive_n; Tbrake_n; delta_n]; 

% input limits
u_lim = [Tdrive_lim; Tbrake_lim; delta_lim];
u_min = u_lim(:,1);
u_max = u_lim(:,2);

%Check number of inputs
if nu ~= length(u_s) 
    error('Number of inputs is not consistent');
end

%Constraints on the rate of inputs
duk_ub = duk_ub./u_s;
duk_lb = duk_lb./u_s;

%%%%%%% Additional decision variables of the NLP other than the states and
%%%%%%% inputs scaling and bounds %%%%%%%%%%%%%

ny = 1; % Number of aux variables

% longitudinal load transfer [N]
ltx_s = vp.m*vp.g*vp.hcg/vp.l;
% ltx = ltx_s * ltx_n;

% scaling factors
y_s = [ltx_s]; 
% aux. variables vector (scaled) - decision variable
% y = [ltx_n]; 

% aux. variables limits
ltx_lim = 1/ltx_s * [-vp.m*vp.g vp.m*vp.g];

y_lim = [ltx_lim];
y_min = y_lim(:,1);
y_max = y_lim(:,2);

%Check number of inputs
if ny ~= length(y_s) 
    error('Number of aux. variables is not consistent');
end

%Symbolic variables that are not decision variables of the NLP - defined as such because their value changes (it is like a variable value parameter)

% kappa > 0 for left turns

pv = [kappa]; % collect variable parameters

% %% Optimal Control Problem -Objetive function
% 
% % Change of variable
% sf = (1-n*kappa)/(vx*cos(epsi)-vy*sin(epsi));
% 
% % Objective function
% objective_function = @(x) x(1:n_states:n_states*N_points,:)'*x(1:n_states:n_states*N_points,:);
% 
% %% Optimal Control Problem - Path Constraints
% 
% %-brake and throttle overlap
% BrTh = Tdrive_n*Tbrake_n; 
% 
% %-longitudinal load transfer
% ltx_eq = ((fx_f*cos(delta) - fy_f*sin(delta) + fx_r - f_drag)*vp.hcg/vp.l - ltx)/ltx_s;
% % ltx_eq = (dvx*vp.m*vp.hcg/vp.l - ltx)/ltx_s;
% 
% %-friciton circle
% mu_lim_f = mu_f^2 - (fx_f^2+fy_f^2)/fz_f^2;
% mu_lim_r = mu_r^2 - (fx_r^2+fy_r^2)/fz_r^2;
% 
% % Collect constraints
% hnames = {'mu_lim_f', 'mu_lim_r', 'ltx_eq', 'BrTh'};
% h = [mu_lim_f; mu_lim_r; ltx_eq; BrTh];
% h_lb = [0; 0; 0-OPT_e; 0-OPT_e];
% h_ub = [inf; inf; 0+OPT_e; 0+OPT_e];
% 
% % Path constraints
% h_eq = Function('h_eq', {x, u, y, pv}, {h}, {'x','u','y','pv'}, {'h'}); 
% 
% %% NLP - Collocation, polynomial coefficients
% 
% % Collocation points in normalised interval [0,1]
% tau = collocation_points(OPT_d, 'legendre');
% 
% % Collocation linear maps
% [C,D,B] = collocation_coeff(tau); %see >>help collocation_coeff (Casadi documentation)
% 
% 
% %% NLP - Collocation, discretisation
% %Create a discrete set of points for collocation
% 
% %-number of grid intervals (the number of points, Xk, is N+1)
% N = round(track.s(end)/OPT_ds); 
% 
% %-value of the independent variable at the discretisation points. 
% s_knot = linspace(min(track.s),max(track.s),N+1); 
% 
% %-length of the discretisation interval
% dsk = diff(s_knot); % Note that the size of s_knot is N+1 whereas for dsk it is N
% 
% %-value of the independent variable at the collocation points
% s_col = kron(dsk,tau)+kron([0 cumsum(dsk(1:end-1))],ones(1,OPT_d)); %value of s at the collocation points (in between grid points)
% 
% %-full array of points including knot (grid) points and collocation points
% s_full = kron(s_knot(1:end-1), [1 zeros(1,OPT_d)]) + reshape([zeros(1,N); reshape(s_col,OPT_d,N)],1,[]); %collect all values of the independent variable at the grid points and collocation points
% s_full(end+1) = s_knot(end);
% 
% % Value of varying parameters (in this case only the curvature)
% k_knot = interp1(track.s, track.k, s_knot); 
% k_col = interp1(track.s, track.k, s_col);
% k_full = interp1(track.s, track.k, s_full);
% 
% %-collect values of variable parameters; dimensions: nv by N
% pv_knot = [k_knot(:)']; 
% pv_col = [k_col(:)'];
% pv_full = [k_full(:)'];
% 
% %% NLP - initial guesses (constant values)
% %-States
% vx_0 = 15*ones(1,N+1);
% vy_0 = OPT_e*ones(1,N+1); % a small value is used instead of zero to help the solver
% r_0 = OPT_e*ones(1,N+1); 
% n_0 = OPT_e*ones(1,N+1);  
% epsi_0 = zeros(1,N+1); 
% Om_f_0 = vx_0/vp.Rw; 
% Om_r_0 = vx_0/vp.Rw;
% %-Inputs
% Tdrive_0 = 1000*ones(1,N+1); Tbrake_0 = 0*ones(1,N+1); delta_0 = OPT_e*ones(1,N+1);
% %-Aux variables
% ltx_0 = zeros(ny,N+1);
% 
% % Collect initial guesses
% x0 = [vx_0; vy_0; r_0; n_0; epsi_0; Om_f_0; Om_r_0]./x_s;
% u0 = [Tdrive_0; Tbrake_0; delta_0]./u_s;
% % xc0 = interp1(s_knot,x0',s_col)'; %linear interpolation between x0
% xc0 = reshape(kron(x0(:,1:end-1),ones(1,OPT_d)),OPT_d*N*nx,1); %constant interpolation between x0
% y0 = [ltx_0]./y_s;
% 
% %% NLP - formulation
% 
% % Decision variables
% Xk = SX.sym('Xk', nx,N+1);       % States
% Uk = SX.sym('Uk', nu,N+1);       % Inputs
% Yk = SX.sym('Yk', ny,N+1);       % Aux variables
% Xkj = SX.sym('Xkj', nx,N*OPT_d); % Helper states for the collocation constraints
% 
% % Linear interpolation
% duk = diff(Uk')'./repmat(dsk,nu,1); %derivative of the input at each interval (du/ds)
% dyk = diff(Yk')'./repmat(dsk,ny,1); %derivative of the aux. variables at each interval (dy/ds)
% % Second derivatives (for regularisarion and minimising oscillations)
% duk2 = diff(duk')'; duk2 = [duk2 duk2(:,end)];
% dyk2 = diff(dyk')'; dyk2 = [dyk2 dyk2(:,end)]; 
% 
% % Scaling of the objective function
% J_s = 1; %Note that using 's_full(end)/vx_0' here instead of 1 would make the objective function be close to 1;
% 
% %% NLP - formulation, boundary conditions
% 
% x0_min = max(x_min, Xi./x_s-OPT_e); x0_max = min(x_max, Xi./x_s+OPT_e);
% xf_min = max(x_min, Xf./x_s-OPT_e); xf_max = min(x_max, Xf./x_s+OPT_e);
% 
% %-Initial state
% gb = {Xk(:,1)}; lbg = x0_min; ubg = x0_max; %Note that gb, lbg and ubg are being initialised here
% 
% %-Final state
% gb = [gb(:); {Xk(:,end)}]; lbg = [lbg; xf_min]; ubg = [ubg; xf_max];
% 
% %% NLP - formulation, collocation constraints, path constraints and objective function
% 
% % Monitored variables
% dt_opt = cell(1,N);
% 
% % Loop over discretisation points to create collocation constraints
% gck = {}; % Collector for collocation constraints
% J = 0; % Initialise objective function
% 
% for k = 0:N-1
%     % Collocation constraints
%     Z = [Xk(:,k+1) Xkj(:,OPT_d*k+(1:OPT_d))]; % Concatenate states
%     %-Dynamics
%     %%-calculate derivatives of the approximating polynomial at the collocation points
%     dPi = Z*C; 
%     %%-calculate derivatives of the system at the collocation points
%     switch OPT_uinter 
%         case 'constant'
%             [dXkj, Qk] = f_dyn(Xkj(:,OPT_d*k+(1:OPT_d)), Uk(:,k+1), Yk(:,k+1), pv_col(:,OPT_d*k+(1:OPT_d))); 
%         case 'linear'
%             [dXkj, Qk] = f_dyn(Xkj(:,OPT_d*k+(1:OPT_d)), Uk(:,k+1)+kron(duk(:,k+1),tau), Yk(:,k+1)+kron(dyk(:,k+1),tau), pv_col(:,OPT_d*k+(1:OPT_d)));
%         otherwise
%             error('Choose "linear" or "constant" for the interpolation method of the inputs');
%     end
%     %-State of approximating polynomial the end of the colocation interval
%     Xk_end = Z*D;
% 
%     gck = [gck(:); {dsk(k+1)*dXkj(:) - dPi(:)}; {Xk_end-Xk(:,k+2)}]; % Add collocation constraints
% 
%     % Integrate quadrature function
%     J = J + Qk*B*dsk(k+1)/J_s + sumsqr(rdu.*duk(:,k+1)) + sumsqr(rdy.*dyk(:,k+1)) + sumsqr(rdu2.*duk2(:,k+1)) + sumsqr(rdy2.*dyk2(:,k+1)) + sumsqr(ru*Uk(2,k+1));
% 
%     % Collect contribution to time of each interval
%     dt_opt{k+1} = Qk*B*dsk(k+1); 
% end
% 
% % Path constraints
% ghk = h_eq(Xk, Uk, Yk, pv); ghk = {ghk(:)};
% 
% % Rate of inputs constraints
% Sfk = f_sf(Xk,k_knot); % calculate Sf to 'undo' the change of independent variable
% duk_t = duk./repmat(Sfk(1:end-1),nu,1); % calculate time derivatives: du/dt = du/ds * 1/sf = du/ds * ds/dt
% gduk = {duk_t(:)};
% 
% 
% %% NLP - define NLP problem
% %-decision variables
% % % w = [Xk(:); Uk(:); Yk(:); Xkj(:)]  Collect all the decision variables [states; inputs; aux. variables; helper states]
% lbw = [repmat(x_min(:),N+1,1); repmat(u_min(:,1),N+1,1); repmat(y_min(:),N+1,1); repmat(x_min(:),N*OPT_d,1)]; % Lower bounds
% ubw = [repmat(x_max(:),N+1,1); repmat(u_max(:,1),N+1,1); repmat(y_max(:),N+1,1); repmat(x_max(:),N*OPT_d,1)]; % Upper bounds
% w0 = [x0(:); u0(:); y0(:); xc0(:)]; % Collect initial guesses for the decision variables
% 
% %-constraints
% g = [gb(:);gck(:);ghk(:);gduk(:)]; g = vertcat(g{:}); %[boundary conditions; collocation constraints; path constraints; rate of inputs]
% lbg = [lbg; zeros((OPT_d+1)*N*nx,1);repmat(h_lb,N+1,1);repmat(duk_lb,N,1)]; % Lower bounds 
% ubg = [ubg; zeros((OPT_d+1)*N*nx,1);repmat(h_ub,N+1,1);repmat(duk_ub,N,1)]; % Upper bounds 
% 
% elapsedTime(end+1) = toc - elapsedTime(end);
% %% NLP - solve 
% 
% % Solve the NLP
% [sol, fval] = fmincon(objective_function, x0, [], [], [], [], lbg, ubg, constraint_function, options);
% 
% elapsedTime(end+1) = toc - elapsedTime(end);
% 
% %% Postprocessing - Collect NLP variables
% %Retrieve decision variables from the solution
% global data;
% 
% %-Independent variables
% data.s_full = s_full;
% % data.s_knot = s_knot;
% data.k_full = k_full;
% % data.k_knot = k_knot;
% 
% %-Collect decision variables in numeric array
% data.w_opt = full(sol);
% % The shape of w_opt is w_opt = [Xk(:); Uk(:); Yk(:); Xkj(:)];
% % length(Xk) = nx*(N+1);
% % length(Uk) = nu*(N+1);
% % length(Yk) = ny*(N+1);
% % legth(Xkj) = nx*d*N;
% %-Note scaling is undone here too
% data.x_opt = reshape(data.w_opt(1:nx*(N+1)),nx,N+1).*x_s;
% data.u_opt = reshape(data.w_opt(nx*(N+1)+(1:nu*(N+1))),nu,N+1).*u_s;
% data.y_opt = reshape(data.w_opt(nx*(N+1)+nu*(N+1)+(1:ny*(N+1))),ny,N+1).*y_s;
% data.xc_opt = reshape(data.w_opt(nx*(N+1)+nu*(N+1)+ny*(N+1)+1:end),nx,N*OPT_d).*x_s;
% 
% %% Postprocessing - Reconstruct solution
% % Collect values of x at all interpolation points 
% %States, x_full = [X1 X11...X1j...X1d,..., Xk Xk1...Xkj...Xkd,..., XN XN1..XNj...XNd, XN+1]
% data.x_full = kron(data.x_opt(:,1:end-1), [1 zeros(1,OPT_d)]) + reshape([zeros(nx,N); reshape(data.xc_opt,nx*OPT_d,N)],nx,[]);
% data.x_full(:,end+1) = data.x_opt(:,end);
% %Inputs and Aux. variables
% switch OPT_uinter
%     case 'constant'
%         data.u_full = interp1(s_knot', data.u_opt', s_full, 'previous')'; % for constant inputs
%         data.y_full = reshape(interp1(s_knot', data.y_opt', s_full, 'previous')',ny,[]); % for constant inputs
%     case 'linear'
%         data.u_full = interp1(s_knot, data.u_opt', s_full, 'linear')'; % for linear inputs
%         data.y_full = reshape(interp1(s_knot, data.y_opt', s_full, 'linear')',ny,[]); % for linear inputs
%     otherwise
%         error('Choose "linear" or "constant" for the interpolation method of the inputs');
% end
% 
% %% Postprocessing - Collect additional data
% 
% % Time
% %-time at grid points
% dt_opt_val = cell(N,1);
% for i=0:N-1
%     f_t_opt = Function('f_t_opt',{Xkj(:,(i*OPT_d)+(1:OPT_d)), Uk(:,i+1), Yk(:,i+1)},{dt_opt{i+1}}); %Create a casadi function to evaluate dt_opt
%     dt_opt_val{i+1} = f_t_opt(data.xc_opt(:,i*OPT_d+(1:OPT_d))./x_s,data.u_opt(:,i+1)./u_s, data.y_opt(:,i+1)./y_s); %Evaluate dt_opt at the solution
% end
% data.t_opt = [0 cumsum(full([dt_opt_val{:}]))]; %Time at each grid point
% 
% % Track
% data.track0 = track; %Track before discretisation
% data.track.s = s_full;
% data.track.k = k_full;
% %%-calculate track coordinates at grid points
% if ~isfield(data.track0,'x') || ~isfield(data.track0,'y')
%     [data.track.x, data.track.y] = curv2cart(data.track.s, data.track.k);
% else
%     data.track.x = interp1(data.track0.s, data.track0.x, data.track.s);
%     data.track.y = interp1(data.track0.s, data.track0.y, data.track.s);
% end
% [data.track.xopt,data.track.yopt] = cartPath(data.track.x,data.track.y, data.x_full(4,:)); %x_full(4) is the distance to the centerline
% [data.track.Xl,data.track.Xr] = trackLimits(data.track.x,data.track.y, x_s(4)*x_max(4)*2+2);
% 
% % Vehicle
% %-aerodynamic forces
% f_aeroF = Function('f_aeroF',{x,u,y,pv}, {[f_drag; f_lift]},{'x','u','y','pv'},{'Ftyres'});
% aeroF = reshape(full(f_aeroF(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),2,1,N+1);
% data.vehicle.f_drag = aeroF(1,1,:);
% data.vehicle.f_lift = aeroF(2,1,:);
% %-tyre forces
% f_Ftyres = Function('f_Ftyres',{x,u,y,pv},{[fx_f, fy_f, fz_f; fx_r, fy_r, fz_r]},{'x','u','y','pv'},{'Ftyres'});
% Ftyres = reshape(full(f_Ftyres(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),2,3,N+1);
% data.vehicle.fx_f = Ftyres(1,1,:);
% data.vehicle.fy_f = Ftyres(1,2,:);
% data.vehicle.fz_f = Ftyres(1,3,:);
% data.vehicle.fx_r = Ftyres(2,1,:);
% data.vehicle.fy_r = Ftyres(2,2,:);
% data.vehicle.fz_r = Ftyres(2,3,:);
% %-slip angles
% f_slipAng = Function('f_slipAng',{x,u,y,pv},{[sa_f; sa_r]},{'x','u','y','pv'},{'slipAng'});
% slipAng = reshape(full(f_slipAng(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),2,1,N+1);
% data.vehicle.sa_f = slipAng(1,1,:);
% data.vehicle.sa_r = slipAng(2,1,:);
% %-slip ratios
% f_slipX = Function('f_slipX',{x,u,y,pv},{[sx_f; sx_r]},{'x','u','y','pv'},{'slipX'});
% slipX = reshape(full(f_slipX(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot)),2,1,N+1);
% data.vehicle.sx_f = slipX(1,1,:);
% data.vehicle.sx_r = slipX(2,1,:);
% 
% % Constraints
% hval = full(h_eq(data.x_opt./x_s, data.u_opt./u_s, data.y_opt./y_s, pv_knot));
% for i=1:length(hnames)
%     data.constraints.(hnames{i}) = hval(i,:);
% end
% 
% %% Postprocessing - Log data with Simulation Data Inspector
% 
% % Initialise figures and SDI
% % Simulink.sdi.clear; %clears all data in SDI **USE CAREFULLY**
% %-define container for external figures
% global figures;
% figures = struct();
% %-sync data cursors between SDI and external figures
% figures.callbackID = Simulink.sdi.registerCursorCallback(@(t1,t2)onCursorMove(t1,t2));
% %-initialise external figures
% %%-track
% figures.track.fig = figure('Name','Circuit Map'); clf
% figures.track.ax = axes;
% updateTrackPlot(nan,nan);
% 
% % Create time series objects for sdi
% %-states, inputs and aux. variables
% for i = 1:length(x)
%     aux = x(i); name = erase(aux.name,'_n'); %get name of variable
%     data.sdi.solData.states.(name) = timeseries(data.x_full(i,:), s_full,'name',name); %create timeseries object
% end
% for i = 1:length(u)
%     aux = u(i); name = erase(aux.name,'_n');
%     data.sdi.solData.inputs.(name) = timeseries(data.u_full(i,:), s_full,'name',name);
% end
% for i = 1:length(y)
%     aux = y(i); name = erase(aux.name,'_n'); 
%     data.sdi.solData.aux.(name) = timeseries(data.y_full(i,:), s_full,'name',name); 
% end
% data.sdi.solData.time = timeseries(data.t_opt, s_knot, 'name', 'time');
% clear aux name
% %-track data
% data.sdi.track.s = timeseries(s_full,s_full, 'name', 's');
% data.sdi.track.k = timeseries(k_full,s_full, 'name', 'k');
% data.sdi.track.x = timeseries(data.track.x,s_full, 'name', 'x');
% data.sdi.track.y = timeseries(data.track.y,s_full, 'name', 'y');
% %-vehicle data
% %%-parameters (just as a way to store the values that were used for each simulation)
% fn = fieldnames(vp); %get names of vehicle parameters
% for i = 1:length(fn)
%     if isa(vp.(fn{i}),'double')
%         data.sdi.vehicle.params.(fn{i}) =  timeseries(repmat(vp.(fn{i}),1,length(s_knot)),s_knot);
%     end
% end
% clear fn
% %%-aerodynamic forces
% data.sdi.vehicle.f_drag = timeseries(data.vehicle.f_drag, s_knot, 'name', 'f_drag');
% data.sdi.vehicle.f_lift = timeseries(data.vehicle.f_lift, s_knot, 'name', 'f_lift');
% %%-tyre forces
% data.sdi.vehicle.fx_f = timeseries(data.vehicle.fx_f, s_knot, 'name', 'fx_f');
% data.sdi.vehicle.fy_f = timeseries(data.vehicle.fy_f, s_knot, 'name', 'fy_f');
% data.sdi.vehicle.fz_f = timeseries(data.vehicle.fz_f, s_knot, 'name', 'fz_f');
% data.sdi.vehicle.fx_r = timeseries(data.vehicle.fx_r, s_knot, 'name', 'fx_r');
% data.sdi.vehicle.fy_r = timeseries(data.vehicle.fy_r, s_knot, 'name', 'fy_r');
% data.sdi.vehicle.fz_r = timeseries(data.vehicle.fz_r, s_knot, 'name', 'fz_r');
% %%-slip angles
% data.sdi.vehicle.sa_f = timeseries(data.vehicle.sa_f, s_knot, 'name', 'sa_f');
% data.sdi.vehicle.sa_r = timeseries(data.vehicle.sa_r, s_knot, 'name', 'sa_r');
% %%-slip ratios
% data.sdi.vehicle.sx_f = timeseries(data.vehicle.sx_f, s_knot, 'name', 'sx_f');
% data.sdi.vehicle.sx_r = timeseries(data.vehicle.sx_r, s_knot, 'name', 'sx_r');
% %-path constraints
% for i = 1:length(hnames)
%     data.sdi.constraints.(hnames{i}) = timeseries(data.constraints.(hnames{i}), s_knot, 'name', hnames{i});
% end
% 
% % Import data to SDI
% newRun = Simulink.sdi.Run.create;
% newRun.Name = ['(version)_' 'ds' num2str(OPT_ds) '_' OPT_uinter '_sol:' num2str(data.t_opt(end)) 's_X0=' num2str(Xi(:)') '_Xf=' num2str(Xf(:)') '_ET:' num2str(elapsedTime(3)) 's_' datestr(datetime)];
% solutionData = data.sdi.solData;
% trackData = data.sdi.track;
% vehicleData = data.sdi.vehicle;
% pathConstraints = data.sdi.constraints;
% add(newRun, 'vars', solutionData, trackData, vehicleData, pathConstraints);
% data.sdi.run = newRun;
% clear newRun solutionData trackData
% %-plot data in SDI
% plotSDI;
% 
% %%
% elapsedTime(end+1) = toc - elapsedTime(end);
% disp(' ')
% disp('Elapsed time:')
% disp(table(elapsedTime(2),elapsedTime(3),elapsedTime(4),'VariableNames',{'Transcription','Solution','Post-processing'}))
% 
% disp('Objective function value:')
% disp(full(sol.f))
% 
% disp('Lap time:')
% disp(data.t_opt(end))
