function [dx_n,F]=vehicle(x_n,u_n,y_n,x_s,u_s,y_s,vp,kappa)

% Nonlinear dynamic model of a road vehicle with seven states: battery charge,
% 2-D velocity, yaw rate, curvilinear absissa, transversal position,
% heading angle, steering angle. Nonlinear lateral tyre forces;
% saturation on battery SOC and battery output current, maximum engine power, saturation on 
% front wheel steering angle and steering rate. Partial de-coupling of longitudinal and
% lateral dynamics (assume longitudinal speed varies slowly with respect to
% lateral dynamics)
%
% Inputs:   
%           x       (model state)
%           u       (driving torque, braking torque, steering angle)          
%           vp      (model parameters list)
%           kappa   (circuit curvature)
%
% Outputs:  x_dot   (derivative of the state with respect to time)
%           F       (longitudinal and lateral forces of the vehicle)

%% States and inputs

% States
x=x_s*x_n';
vx      =       x(1,1);     % body x velocity (m/s)
vy      =       x(2,1);     % body y velocity (m/s)
r       =       x(3,1);     % yaw rate [rad/s]
n       =       x(4,1);     % transversal displacement (m)
epsi    =       x(5,1);     % angle to centreline tangent direction [rad]
Om_f    =       x(6,1);     % angular velocity front tyre [rad/s]      
Om_r    =       x(7,1);     % angular velocity rear tyre [rad/s]

% Inputs
u=u_s*u_n;
Tdrive            =       u(1,1);     % driving torque (Nm)
Tbrake            =       u(2,1);     % front braking torque (Nm)
delta             =       u(1,1);     % Steering rate (rad/s)

% longitudinal load transfer [N]
ltx = y_s * y_n;

%% Vehicle model - equations
%Calculate variables of the system other than the states and inputs

% Resistance forces
%-aerodynamic forces [N]
f_drag = 0.5*vp.rho*vp.Cd*vp.A*vx^2;
f_lift = 0.5*vp.rho*vp.Cl*vp.A*vx^2;

%-rolling resistance [N]
vth = 1; %(m/s) threshold speed for rolling resistance
rollCoeff = vp.f*tanh(4*vx/vth); % 
f_roll_f = rollCoeff*vp.m*vp.g*vp.l_r/vp.l; %Note that the load transfer is being neglected
f_roll_r = rollCoeff*vp.m*vp.g*vp.l_f/vp.l;

% Tyres
%-slip
%%-slip angles [rad]
sa_f = delta - atan((vp.l_f*r+vy)/vx);
sa_r = atan((vp.l_r*r-vy)/vx);
%%-slip ratio
v_f = sqrt((vy+vp.l_f*r)^2 + vx^2); v_r = sqrt((vy-vp.l_f*r)^2 + vx^2);
v_fx = v_f*cos(sa_f); v_rx = v_r*cos(sa_r);
sx_f = (vp.Rw*Om_f - v_fx)/v_fx;
sx_r = (vp.Rw*Om_r - v_rx)/v_rx;
%-forces
%%-fx, vertical tyre forces [N]
fx_f = vp.m*vp.g*vp.l_r/vp.l - ltx  + f_lift/2; %Longitudinal load transfer -> [Acceleration +; Brake -]
fx_r = vp.m*vp.g*vp.l_f/vp.l + ltx  + f_lift/2;

mu_f = vp.tyre.mu + vp.tyre.pD2*(fx_f-vp.tyre.Fx0)/vp.tyre.Fx0;
mu_r = vp.tyre.mu + vp.tyre.pD2*(fx_r-vp.tyre.Fx0)/vp.tyre.Fx0;

%%-fx, longitudinal tire forces  [N]
%·linear tyres
% fx_f = vp.Cx*sx_f;
% fx_r = vp.Cx*sx_r;
%·magic formula
%%·parameters
Bx = vp.tyre.bx;
Cx = vp.tyre.cx; 
Ex = vp.tyre.ex;
Dx_f = mu_f*fx_f; Dx_r = mu_r*fx_r;
%%·Fx(sx)
fx_f = Dx_f*sin(Cx*atan(Bx*sx_f-Ex*(Bx*sx_f-atan(Bx*sx_f)))) - f_roll_f;
fx_r = Dx_r*sin(Cx*atan(Bx*sx_r-Ex*(Bx*sx_r-atan(Bx*sx_r)))) - f_roll_r;

%%-fy, lateral tyre forces [N]
%·linear tyres
% fy_f = vp.Cy*sa_f;
% fy_r = vp.Cy*sa_r;
%·magic formula
%%·parameters
By = vp.tyre.by;
Cy = vp.tyre.cy; 
Ey = vp.tyre.ey;
Dy_f = mu_f*fx_f; Dy_r = mu_r*fx_r;
%%-Fy(dx)
fy_f = Dy_f*sin(Cy*atan(By*sa_f-Ey*(By*sa_f-atan(By*sa_f))));
fy_r = Dy_r*sin(Cy*atan(By*sa_r-Ey*(By*sa_r-atan(By*sa_r))));

%Wheel torque
T_f = Tbrake*vp.brkB; 
T_r = Tdrive + Tbrake*(1-vp.brkB);

% Change of independent variable
sf = (1-n*kappa)/(vx*cos(epsi)-vy*sin(epsi));

%% Vehicle model - state derivatives
%Define derivatives of the state-space model

% State derivatives equations
dvx = (fx_r + fx_f*cos(delta)-fy_f*sin(delta) - f_drag + vp.m*vy*r)*sf/vp.m;
dvy = (fy_r + fy_f*cos(delta)+fx_f*sin(delta) - vp.m*vx*r)*sf/vp.m;
dr = (fy_f*vp.l_f*cos(delta)+fx_f*vp.l_f*sin(delta) - fy_r*vp.l_r)*sf/vp.I_x;
dn = (vx*sin(epsi)+vy*cos(epsi))*sf;
depsi = sf*r-kappa;
dOm_f = sf * (T_f - fx_f*vp.Rw)/vp.Jw;
dOm_r = sf * (T_r - fx_r*vp.Rw)/vp.Jw;

% State derivatives vector (scaled to match the scaled states vector)
dx_n = [dvx; dvy; dr; dn; depsi; dOm_f; dOm_r]./x_s;

% all forces
F =  [f_drag;f_lift;sx_f;sx_r;fx_f;fx_r;fy_f;fy_r;T_f;T_r];

