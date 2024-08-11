% "a0_vehicle_parameters.m" - initializes the constant vehicle parameters. These
% are all stored in a structure called "chassis".

chassis.m       = 1420;               % vehicle mass (kg)
chassis.lf      = 1.323;               % distance between center of gravity and front axle (m)
chassis.lr      = 1.377;               % distance between center of gravity and rear axle (m)
chassis.h       = 0.38;               % distance between center of gravity and road (m)
chassis.wv       = 1.75;               % vehicle width (m)
chassis.Izz     = 1960;               % vehicle moment of inertia around z axis (kg*m^2)
chassis.rho_air = 1.2;               % air density (kg/m^3)
chassis.A_f     = 1.95;               % vehicle front area (m^2)
chassis.Cd      = 0.75;               % aerodynamic drag coefficient (-)
chassis.Cl      = 1.45;               % aerodynamic lift coefficient (-)
chassis.brake_dist = 0.625;             % Brake distribution (if >0.5, it is greater on the front wheel) (-)