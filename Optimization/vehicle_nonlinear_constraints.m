function [c, ceq] = vehicle_nonlinear_constraints(z0,u,th,circuit,min_values,max_values)
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

% Inputs
omega_delta      =       u(1,1);     % Steering rate (rad/s)
Pg               =       u(2,1);     % PMSG power (W)
Pb               =       u(3,1);     % Battery power (W)
Ph               =       u(4,1);     % Braking power (W)

% Maximum values 
ib_max           =       max_values(1,1);       % Maximum battery current (A)
Qb_max           =       max_values(2,1);       % Maximum battery SOC (Ah)

% Minimum values
ib_min           =       min_values(1,1);       % Minimum battery current (A)
Qb_min           =       min_values(2,1);       % Minimum battery SOC (Ah)

%% Compute the state values
% States
N_points = length(circuit(1,:));
rho     = circuit(1,:);
w_track = circuit(2,:);
w_track_min = min(w_track);
delta_s = circuit(9,:);
z       = zeros(8, N_points);
z(:,1)  = z0;

for s=1:N_points-1
    [z_prime, ~] = vehicle(z(:,s),u(:,s),th,rho(1,s));
    z(:,s+1)     = z(:,s) + delta_s(s)*z_prime;              % With FFD (we will need to change this)  
end

% States
t        =       z(1,:);     % curvilinear absissa (m)
Qb       =       z(2,:);     % battery SOC [Ah]
vx       =       z(3,:);     % body x velocity (m/s)
vy       =       z(4,:);     % body y velocity (m/s)
omega    =       z(5,:);     % yaw rate (rad/s)
n        =       z(6,:);     % transversal displacement (m)
alpha    =       z(7,:);     % heading angle (rad)       
delta    =       z(8,:);     % steering angle (rad)

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
Sf      =       rb*min(Pt+Ph,0)./vx;                                 % Longitudinal driving/braking force on the front (N)
Sr      =       (Pt+Ph-rb*min(Pt+Ph,0))./vx;                         % Longitudinal driving/braking force on the front (N)
Nf      =       m*g*b/(a+b)-h/(a+b)*(Sr+Sf);                        % vertical force on front axle (N)
Nr      =       m*g*a/(a+b)+h/(a+b)*(Sr+Sf);                        % vertical force on rear axle (N)
lambda_f=       delta-(a*omega+vy)./vx;                              % front sideslip angle (rad)
lambda_r=       delta-(b*omega-vy)./vx;                              % rear sideslip angle (rad)
Ff      =       Kf*lambda_f.*Nf;                                     % front lateral force (N) 
Fr      =       Kr*lambda_r.*Nr;                                     % front lateral force (N) 
Fxd     =       0.5*rho_air*Cd*A_f*vx.^2;                            % front aerodynamic resistance (N)

%% Constraints definition

% Nonlinear Inequality Constraints
c = [Fr.^2 + Sr.^2 - (mux_r*Nr).^2;   % Constraint 1: Fr^2 + Sr^2 - (mux_r*Nr)^2 <= 0
     Ff.^2 + Sf.^2 - (mux_f*Nf).^2;   % Constraint 2: Ff^2 + Sf^2 - (mux_r*Nr)^2 <= 0
     ib - ib_max;                     % Constraint 3: ib - ib_max <= 0       
     - ib + ib_min;                   % Constraint 4: -ib + ib_min <= 0
     Qb - Qb_max;                     % Constraint 5: Qb - Qb_max <= 0
     -Qb + Qb_min;                    % Constraint 6: -Qb + Qb_min <= 0
     Ph;                              % Constraint 9: Ph <= 0
     n - w_track_min/2;               % Constraint 10: n - w_track/2 <= 0
     - w_track_min/2 + n];            % Constraint 11: - w_track/2 + n <= 0


% Absence of nonlinear Equality Constraints
    ceq = [];
end