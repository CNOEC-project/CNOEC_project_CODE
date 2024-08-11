function v = vehicle_cost_and_constraints_BFGS(z_full,u_vec,N,chassis,tyre,s_col,z_norm_factor,u_norm_factor,k_col,C_col,D_col,nz_grid_points,OPT_d,B_col)
    %VEHICLE_COST_AND_CONSTRAINTS_BFGS computes the overall cost function and the nonlinear equality and inequality constraints for the optimization problem.  
    %
    % Inputs:   z_full          (Vector containing all the states stacked at the grid and collocation points)
    %           u_vec           (Vector containing all the inputs stacked at the grid points)
    %           N               (Total number of collocation points)
    %           chassis         (Vehicle parameters)
    %           tyre            (Tyre parameters)
    %           s_col           (curvilinear absissa at the collocation points)
    %           z_norm_factor   (states normalization factor)
    %           u_norm_factor   (inputs normalization factor)
    %           k_col           (curvature at collocation points)
    %           B_col           (constant matrix for collocation)
    %           nz_grid_points  (Total number of states at the grid points)
    %           OPT_d           (Degree of interpolating legendre polynomial)
    %
    % Outputs:  v               (cost function, equality constraints, inequality constraints)

    % Compute variables of interest  
    dsk = diff(s_col);          % Circuit discretization step (m) 
    n_states = 7;               % Number of states per circuit point
    n_inputs = 3;               % Number of inputs per circuit point
    F = 0;                      % Initialize the objective function 

    % Extract the states at the collocation points
    z_col_vec_norm = z_full(nz_grid_points+1:end);
    
    % Extract the states at the grid points
    z_grid_vec_norm = z_full(1:nz_grid_points);

    z_col  = reshape(z_col_vec_norm, n_states, OPT_d*N).*z_norm_factor;     % Reshape z_col_vec_norm into a matrix  
    z_grid = reshape(z_grid_vec_norm, n_states, N+1).*z_norm_factor;        % Reshape z_grid_vec_norm into a matrix
    u_grid      = reshape(u_vec, n_inputs, N+1).*u_norm_factor;             % Reshape u_vec into a matrix
    tprime_kj = zeros(1,OPT_d);                                             % Initialize the vector of time variation at collocation points

    % Inputs
    T_drive_vec         =       u_vec(1:n_inputs:end);                      % Driving torque (Nm)
    T_brake_vec         =       u_vec(2:n_inputs:end);                      % Braking torque (Nm)
    delta_vec           =       u_vec(3:n_inputs:end);                      % Steering angle (rad)
    
    [Fz_f,Fz_r,Fx_f,Fx_r,Fy_f,Fy_r,mu_f,mu_r] = friction_ellipse_forces(z_grid,u_grid,chassis,tyre);
    
    dXkj = zeros(n_states,OPT_d);                                           % Initialize the vector of state derivatives at collocation points

    g = [T_drive_vec.*T_brake_vec];                                              % Nonlinear equality constraint matrix

    % Nonlinear inequality constraint matrix
    h = [(mu_f.^2-(Fx_f.^2+Fy_f.^2)./Fz_f.^2)';                                % Friction ellipse - front wheel
        (mu_r.^2-(Fx_r.^2+Fy_r.^2)./Fz_r.^2)'];                                % Friction ellipse - rear wheel 

    for kk = 0:N-1
        
        % Concatenate normalized states
        Z = [z_grid(:,kk+1)./z_norm_factor       z_col(:,OPT_d*kk+(1:OPT_d))./z_norm_factor]; 
        
        % Calculate derivatives of the approximating polynomial at the collocation points (i.e., the value of the normalized states at the collocation points)
        dPi = Z*C_col; 
        
        % Calculate derivatives of the system at the collocation points by simulating the system dynamics
        for ii=1:OPT_d
            [dXkj_not_normalized, ~, tprime_kj(1,ii)] = vehicle_dynamics(z_col(:,OPT_d*kk+ii), u_grid(:,kk+1),chassis,tyre, k_col(:,OPT_d*kk+ii));
            dXkj(:,ii) = dXkj_not_normalized./z_norm_factor;
        end
        
        % Normalized state of the approximating polynomial the end of the collocation interval
        Xk_end = Z*D_col;
        
        % Stack the constraints in the equality constraints matrix
        g =     [g;
               reshape(dsk(kk+1)*dXkj(:) - dPi(:),[],1);
               Xk_end-z_grid(:,kk+2)./z_norm_factor];

        % Update the objective function
        F = F + tprime_kj*B_col*dsk(kk+1);
    end

    f = F'*F;
    
    v = [f; g; h]; 
end

