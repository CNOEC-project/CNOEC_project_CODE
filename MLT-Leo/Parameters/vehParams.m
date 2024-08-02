% vehParams.m - Vehicle Parameters 
%
% Create struct storing the value of the parameters used by the vehicle model

%% General parameters of the vehicle
vp.g = 9.81; %(m/s2) acceleration of gravity
vp.m = 1420; %(kg)
vp.I_z = 1960; %(kg*m2)

vp.t = 1.75; %(m) track
vp.l = 2.7; %(m) wheelbase
vp.wB = 0.51; %(pc. front) weight distribution
vp.l_f = (1-vp.wB)*vp.l; %(m)
vp.l_r = vp.wB*vp.l; %(m)
vp.hcg = 0.38; %(m) height of center of gravity

%% Tyre
vp.Rw = 0.355; %(m) wheel radius
vp.Jw = 0.9*2; %(kg*m2) wheel rotational inertia

vp.f = 0.01; %(-)

vp.tyre.mu = 1.3; %(Fi/Fz, i=x,y) limit of adherence for nominal load Fz0

%·linear tyres
% vp.Cx = 30e3; %(N/rad) Longitudinal stiffness -> linear tyre
% vp.Cy = 40e3; %(N/rad) Lateral stiffness -> linear tyre

%·magic formula
vp.tyre.bx = 6; % 
vp.tyre.cx = 2.3; % 
vp.tyre.ex = 0.9; % 

vp.tyre.by = 9; % 
vp.tyre.cy = 2.6; % 
vp.tyre.ey = 1; % 

vp.tyre.Fz0 = 6000; %(N) nominal vertical load
vp.tyre.pD2 = -0.3; % variation of friction with load (mu sensitivity)

%% Aero
vp.A = 1.95; %(m2)
vp.rho = 1.2; %(kg/m3)
vp.Cd = 0.75; %(-)
vp.Cl = 1.45; %(-)

%% Powertrain
vp.Tdrive_max = 1250*2.5; %(Nm)

vp.P_max = 955e3; %(W)

vp.v_max = roots([-0.5*vp.rho*vp.Cd*vp.A, 0, -vp.f*vp.m*vp.g, vp.P_max]); %Calculate v_max for the given power accounting for drag and roll resistance
vp.v_max = vp.v_max(find(vp.v_max == real(vp.v_max),1));

%% Brakes
vp.Tbrake_max = 12e3; %(Nm)
vp.brkB = 0.625; % Brake balance (% front)

