function [zprime,F]=vehicle(z,u,th,rho)
% Nonlinear dynamic model of a road vehicle with seven states: battery charge,
% 2-D velocity, yaw rate, curvilinear absissa, transversal position,
% heading angle, steering angle. Nonlinear lateral tyre forces;
% saturation on battery SOC and battery output current, maximum engine power, saturation on 
% front wheel steering angle and steering rate. Partial de-coupling of longitudinal and
% lateral dynamics (assume longitudinal speed varies slowly with respect to
% lateral dynamics)
%
% Inputs:   
%           z       (model state)
%           u       (steering rate, PMSG power, Battery power, Braking power)          
%           th      (model parameters)
%           rho     (circuit curvature)
%
% Outputs:  z_dot   (derivative of the state with respect to time)
%           F       (longitudinal and lateral forces)

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
t        =       z(1,1);     % curvilinear absissa (m)
Qb       =       z(2,1);     % battery SOC [Ah]
vx       =       z(3,1);     % body x velocity (m/s)
vy       =       z(4,1);     % body y velocity (m/s)
omega    =       z(5,1);     % yaw rate (rad/s)
n        =       z(6,1);     % transversal displacement (m)
alpha    =       z(7,1);     % heading angle (rad)       
delta    =       z(8,1);     % steering angle (rad)

% Inputs
omega_delta      =       u(1,1);     % Steering rate (rad/s)
Pg               =       u(2,1);     % PMSG power (W)
Pb               =       u(3,1);     % Battery power (W)
Ph               =       u(4,1);     % Braking power (W)

%% Compute the powers of different branches
Pe      =       eta_g^-1*Pg;                                        % power delivered by the ICE (W)
Pr      =       eta_r*Pg;                                           % power delivered by the rectifier (W)
Pbl     =       eta_dc^sign(Pb)*Pb;                                 % power delivered by the battery (W)
Pi      =       eta_i^sign(eta_r*Pg+Pb)*(eta_r*Pg+Pb);              % power delivered by the inverter (W)
Pt      =       (eta_i*eta_m*eta_t)^sign(eta_r*Pg+Pb)*eta_r*Pg+Pb;  % power delivered by the transmission (W)

%% Compute the battery current 
ib = Voc/(2*Rb)-sqrt((Voc/(2*Rb))^2-Pbl/Rb);                        % battery current (A)

%% Compute lateral and longitudinal tyre forces
g       =       9.81;                                               % gravity acceleration (m/s^2)
Sf      =       rb*min(Pt+Ph,0)/vx;                                 % Longitudinal driving/braking force on the front (N)
Sr      =       (Pt+Ph-rb*min(Pt+Ph,0))/vx;                         % Longitudinal driving/braking force on the front (N)
Nf      =       m*g*b/(a+b)-h/(a+b)*(Sr+Sf);                        % vertical force on front axle (N)
Nr      =       m*g*a/(a+b)+h/(a+b)*(Sr+Sf);                        % vertical force on rear axle (N)
lambda_f=       delta-(a*omega+vy)/vx;                              % front sideslip angle (rad)
lambda_r=       delta-(b*omega-vy)/vx;                              % rear sideslip angle (rad)
Ff      =       Kf*lambda_f*Nf;                                     % front lateral force (N) 
Fr      =       Kr*lambda_r*Nr;                                     % front lateral force (N) 
Fxd     =       0.5*rho_air*Cd*A_f*vx^2;                            % front aerodynamic resistance (N)

F       =       [Sf;Sr;Nf;Nr;Nr;Ff;Fr;Fxd];

zprime=zeros(8,1);
% Model equations
t_prime      =   (1-n*rho)/(vx*cos(alpha)-vy*sin(alpha));      
zprime(1,1)  =   (1-n*rho)/(vx*cos(alpha)-vy*sin(alpha));                         % t_prime
zprime(2,1)  =   -ib*t_prime;                                                     % Qb_prime 
zprime(3,1)  =   ((Sr+Sf*cos(delta)-Ff*sin(delta)-Fxd)/m+omega*vy)*t_prime;       % vx_prime
zprime(4,1)  =   ((Fr+Sf*sin(delta)+Ff*cos(delta))/m-omega*vx)*t_prime;           % vy_prime
zprime(5,1)  =   ((a*Ff*cos(delta)+Sf*sin(delta)-b*Fr)/Izz)*t_prime;              % omega_prime
zprime(6,1)  =   (vx*cos(alpha)+vy*sin(alpha))*t_prime;                           % n_prime
zprime(7,1)  =   (omega-(vx*cos(alpha)-vy*sin(alpha))/(1-n*rho)*rho)*t_prime;     % alfa_prime
zprime(8,1)  =   omega_delta*t_prime;                                             % delta_prime
