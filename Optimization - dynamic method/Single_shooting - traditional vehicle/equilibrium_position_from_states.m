function u_bar = equilibrium_position_from_states(z_bar,th,rho)
%EQUILIBRIUM_POSITION Computes the constant input u_bar corresponding to the
% equilibrium position z_bar.
% 
% Inputs:
%           z_bar   (chosen equilibrium position)
%           th      (model parameters)
%           rho     (circuit curvature)
% Outputs: 
%           u_bar - constant input corresponding to the equilibrium position

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
rb      =       th(10,1);    % braking front ratio on the front axle [-]
Kr      =       th(11,1);    % rear adimensional cornering stiffness [-]
Kf      =       th(12,1);    % rear adimensional cornering stiffness [-]
rho_air =       th(13,1);    % air density (kg/m^3)
A_f     =       th(14,1);    % vehicle front area (m^2)
Cd      =       th(15,1);    % front aerodynamic drag coefficient
Voc     =       th(16,1);    % open circuit battery voltage (V) 
Rb      =       th(17,1);    % battery internal resistance (ohm)
eta_r   =       th(18,1);    % rectifier efficiency 
eta_dc  =       th(19,1);    % DC/DC step up converter efficiency
eta_i   =       th(20,1);    % DC/AC inverter efficiency
eta_t   =       th(21,1);    % mechanical transmission efficiency
eta_g   =       th(22,1);    % PMS Generator mean efficiency
eta_m   =       th(23,1);    % PMS Motor mean efficiency

% States
t_bar        =       z_bar(1,1);     % curvilinear absissa (m)
Qb_bar       =       z_bar(2,1);     % battery SOC [Ah]
vx_bar       =       z_bar(3,1);     % body x velocity (m/s)
vy_bar       =       z_bar(4,1);     % body y velocity (m/s)
omega_bar    =       z_bar(5,1);     % yaw rate (rad/s)
n_bar        =       z_bar(6,1);     % transversal displacement (m)
alpha_bar    =       z_bar(7,1);     % heading angle (rad)       
delta_bar    =       z_bar(8,1);     % steering angle (rad)

% Inputs
omega_delta      =       @(u) (u(1));     % Steering rate (rad/s)
Pg               =       @(u) (u(2));     % PMSG power (W)
Pb               =       @(u) (u(3));     % Battery power (W)
Ph               =       @(u) (u(4));     % Braking power (W)

%% Compute the powers of different branches
Pe      =       @(u) eta_g^-1*Pg(u);                                        % power delivered by the ICE (W)
Pr      =       @(u) eta_r*Pg(u);                                           % power delivered by the rectifier (W)
Pbl     =       @(u) eta_dc^sign(Pb(u))*Pb(u);                                 % power delivered by the battery (W)
Pi      =       @(u) eta_i^sign(eta_r*Pg(u)+Pb(u))*(eta_r*Pg(u)+Pb(u));              % power delivered by the inverter (W)
Pt      =       @(u) (eta_i*eta_m*eta_t)^sign(eta_r*Pg(u)+Pb(u))*eta_r*Pg(u)+Pb(u);  % power delivered by the transmission (W)

%% Compute the battery current 
ib = @(u) Voc/(2*Rb)-sqrt((Voc/(2*Rb))^2-Pbl(u)/Rb);                        % battery current (A)

%% Compute lateral and longitudinal tyre forces
g       =       9.81;                                               % gravity acceleration (m/s^2)
Sf      =       @(u) rb*min(Pt(u)+Ph(u),0)/vx_bar;                                 % Longitudinal driving/braking force on the front (N)
Sr      =       @(u) (Pt(u)+Ph(u)-rb*min(Pt(u)+Ph(u),0))/vx_bar;                         % Longitudinal driving/braking force on the front (N)
Nf      =       @(u) m*g*b/(a+b)-h/(a+b)*(Sr(u)+Sf(u));                        % vertical force on front axle (N)
Nr      =       @(u) m*g*a/(a+b)+h/(a+b)*(Sr(u)+Sf(u));                        % vertical force on rear axle (N)
lambda_f=       delta_bar-(a*omega_bar+vy_bar)/vx_bar;                              % front sideslip angle (rad)
lambda_r=       delta_bar-(b*omega_bar-vy_bar)/vx_bar;                              % rear sideslip angle (rad)
Ff      =       @(u) Kf*lambda_f*Nf(u);                                     % front lateral force (N) 
Fr      =       @(u) Kr*lambda_r*Nr(u);                                     % front lateral force (N) 
Fxd     =       0.5*rho_air*Cd*A_f*vx_bar^2;                            % front aerodynamic resistance (N)

% F       =       @(u) [Sf;Sr;Nf;Nr;Nr;Ff;Fr;Fxd];


% Model equations  
fun  =  @(u)  [(1-n_bar*rho)/(vx_bar*cos(alpha_bar)-vy_bar*sin(alpha_bar));            % t_bar
                        -1*ib(u);                                                                   % Qb_bar
                         ((Sr(u)+Sf(u)*cos(delta_bar)-Ff(u)*sin(delta_bar)-Fxd)/m+omega_bar*vy_bar);      % vx_bar
                        ((Fr(u)+Sf(u)*sin(delta_bar)+Ff(u)*cos(delta_bar))/m-omega_bar*vx_bar);           % vy_bar
                        ((a*Ff(u)*cos(delta_bar)+Sf(u)*sin(delta_bar)-b*Fr(u))/Izz);                      % omega_bar
                        (vx_bar*cos(alpha_bar)+vy_bar*sin(alpha_bar));                           % n_bar
                        (omega_bar-(vx_bar*cos(alpha_bar)-vy_bar*sin(alpha_bar))/(1-n_bar*rho)*rho);     % alpha_bar
                        omega_delta(u);                                                            % delta_bar
                        ];

u0 = zeros(4,1);                                                    % Initial guess for the solution
u_bar = fsolve(fun, u0); 

end
