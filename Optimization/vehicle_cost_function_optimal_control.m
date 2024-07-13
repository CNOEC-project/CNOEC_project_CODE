function t = vehicle_cost_function_optimal_control(z0,u,th,circuit)
    circuit_points = length(circuit(1,:));
    rho     = circuit(1,:);
    delta_s = circuit(9,:);
    z       = zeros(8, circuit_points);
    z(:,1)  = z0;
    t       = z0(1,1);

    for s=1:circuit_points-1
        [z_prime, ~] = vehicle(z(:,s),u(:,s),th,rho(1,s));
        z(:,s+1)     = z(:,s) + delta_s(s)*z_prime;              % With FFD (we will need to change this)  
        t            = t + z(1,s+1);
    end
end

