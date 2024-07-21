function t = vehicle_cost_function_optimal_control_MS(z,u,th,circuit)
    N_points = length(circuit(1,:));                            % Number of circuit points                      
    rho     = circuit(1,:);                                     % Circuit curvature (1/m)
    delta_s = circuit(9,:);                                     % Circuit discretization step (m)
    z       = zeros(8, N_points);                               % States initialization
    z(:,1)  = z0;
    t       = z0(1,1);

    for s=1:N_points-1
        [z_prime, ~] = vehicle(z(:,s),u(:,s),th,rho(1,s));
        z(:,s+1)     = z(:,s) + delta_s(s)*z_prime;              % With FFD (we will need to change this)  
        t            = t + z(1,s+1);
    end
end

