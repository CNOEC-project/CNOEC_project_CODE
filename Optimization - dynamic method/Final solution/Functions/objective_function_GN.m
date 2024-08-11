function F = objective_function_GN(z_full,u_vec,N,chassis,tyre,s_col,z_norm_factor,u_norm_factor,k_col,B_col,nz_grid_points,OPT_d)
    %OBJECTIVE_FUNCTION_GN computes the overall cost function for the optimization problem.  
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
    % Outputs:  F               (cost function)
    
    % Compute variables of interest  
    dsk = diff(s_col);          % Circuit discretization step (m) 
    n_states = 7;               % Number of states per circuit point
    n_inputs = 3;               % Number of inputs per circuit point
    F = zeros(length(z_full)+length(u_vec),1);           % Initialize the objective function 
    
    % Extract the states at the collocation points
    z_col_vec_norm = z_full(nz_grid_points+1:end);
    
    % Extract the states at the grid points
    z_grid_vec_norm = z_full(1:nz_grid_points);
    
    z_col  = reshape(z_col_vec_norm, n_states, OPT_d*N).*z_norm_factor;             % Reshape z_col_vec_norm into a matrix
    z_grid = reshape(z_grid_vec_norm, n_states, N+1).*z_norm_factor;                % Reshape z_grid_vec_norm into a matrix
    u      = reshape(u_vec, n_inputs, N+1).*u_norm_factor;                          % Reshape u_vec into a matrix
    
    tprime_kj = zeros(1,OPT_d);                                                            % Initialize the vector of time variation at collocation points
    
    for k = 0:N-1
    
        % Calculate time variation at the collocation points
        for jj=1:OPT_d
            [~, ~, tprime_kj(1,jj)] = vehicle_dynamics(z_col(:,OPT_d*k+jj), u(:,k+1), chassis,tyre, k_col(:,OPT_d*k+jj));
        end
    
        % Update the objective function
        F(k+2,1) = tprime_kj*B_col*dsk(k+1);
    end

end

