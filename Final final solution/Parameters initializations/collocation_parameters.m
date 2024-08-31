% Define the collocation method parameters

dsk = 70;                        % Collocation step (m)
d_th = 3;                        % Degree of interpolating polynomials

% Collocation constants 
tau_col = [0.1127,    0.5,    0.8873];

% Collocation matrixes
C_col = [-6, 3, -6; 
        5, -5.72749, 10.164; 
        1.16398, 2, -9.16398; 
        -0.163978, 0.727486, 5];

D_col = [-1; 1.66667; -1.33333; 1.66667];

B_col = [0.277778; 0.444444; 0.277778];


%% Collocation, discretisation
% Create a discrete set of points for collocation

% Number of grid intervals (the number of points, Xk, is N+1)
N = round(track.s(end)/dsk); 

% Value of the independent variable at the discretisation points
s_grid = linspace(min(track.s),max(track.s),N+1); 

% Length of the discretisation interval
dsk = diff(s_grid); 

% Value of the independent variable at the collocation points
s_col = kron(dsk,tau_col)+kron([0 cumsum(dsk(1:end-1))],ones(1,d_th)); % Value of s at the collocation points (in between grid points)

% Full array of points including grid points and collocation points
s_full = kron(s_grid(1:end-1), [1 zeros(1,d_th)]) + reshape([zeros(1,N); reshape(s_col,d_th,N)],1,[]); % Collect all values of the independent variable at the grid points and collocation points
s_full(end+1) = s_grid(end);

% Value of the curvature 
k_grid = interp1(track.s, track.k, s_grid);     % Curvature at grid points 
k_col  = interp1(track.s, track.k, s_col);      % Curvature at collocation points 
k_full = interp1(track.s, track.k, s_full);     % Collect all values of the curvature at the grid points and collocation points