% "tyre_parameters.m" - initializes the constant tyre parameters. These
% are all stored in a structure called "tyre".

tyre.rw      = 0.355;                % wheel radius (m)
tyre.Jw      = 1.8;                  % wheel rotational inertia (kg*m^2)
tyre.mu0_x   = 1.3;                  % Maximum longitudinal friction coefficient with static load (-)
tyre.mu0_y   = 1.3;                  % Maximum lateral friction coefficient with static load (-)
tyre.bx      = 6;                    % Pacejka's magic formula bx coefficient for longitudinal force (-)
tyre.cx      = 2.3;                  % Pacejka's magic formula bx coefficient for longitudinal force (-)
tyre.ex      = 0.9;                  % Pacejka's magic formula ex coefficient for longitudinal force (-)
tyre.by      = 9;                    % Pacejka's magic formula by coefficient for lateral force (-)
tyre.cy      = 2.6;                  % Pacejka's magic formula cy coefficient for lateral force (-)
tyre.ey      = 1;                    % Pacejka's magic formula ey coefficient for lateral force (-)
tyre.Fz0     = 6000;                 % Pacejka's magic formula nominal vertical load (N)
tyre.pD2     = -0.3;                 % Pacejka's magic formula tyre load sensitivity coefficient (-)