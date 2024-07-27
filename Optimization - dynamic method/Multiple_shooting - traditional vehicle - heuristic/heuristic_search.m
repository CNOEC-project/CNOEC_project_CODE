function [u0,z,flag] = heuristic_search(z0,lb_heu,ub_heu,lb_z,ub_z,th,circuit)
    %HEURISTIC_SEARCH Summary of this function goes here
    %   Detailed explanation goes here
    n_inputs = 3;                % Number of inputs
    n_states = 7;                % Number of states

    %% Compute the state values
    % States
    N_points = length(circuit(1,:));                % Number of circuit points 
    rho     = circuit(1,:);                         % Circuit curvature (1/m) 
    w_track = circuit(2,:);                         % Circuit width (m)
    w_track_min = min(w_track);                     % Circuit minimum width (m)
    delta_s = circuit(9,:);                         % Circuit discretization step (m)
    z = zeros(n_states,N_points);                   % Initialize the state vector
    z(:,1) = z0(:,1);                               % Assign the initial state
    
    % Specify the interval within which the solution is to be searched
    omega_delta_range = linspace(lb_heu(1),ub_heu(1),100);
    Tdr_range = linspace(lb_heu(2),ub_heu(2),100);
    Tdf_range = linspace(lb_heu(3),ub_heu(3),100);
    
    % Initialize a 3D cell array with dimensions based on the lengths of omega_delta_range, Tdr_range, and Tdf_range
    grid_u = cell(length(omega_delta_range),length(Tdr_range),length(Tdf_range)); 
    
    % Loop over each dimension of the 3D cell array
    for ii=1:size(grid_u,1)
        for jj=1:size(grid_u,2)
            for kk=1:size(grid_u,3)
                % Assign a vector containing elements from omega_delta_range, Tdr_range, and Tdf_range to each cell
                grid_u{ii,jj,kk} = [omega_delta_range(ii), Tdr_range(jj), Tdf_range(kk)];
            end
        end
    end

    u0 = zeros(n_inputs,N_points);
    for ii=1:size(grid_u,1)
            for jj=1:size(grid_u,2) 
                for kk=1:size(grid_u,3)
                    flag = 0;
                    for s=1:N_points
                        u0(:,s) = grid_u{ii,jj,kk};
                    end

                    for s=1:N_points-1
                        [z_prime, ~] = vehicle(z(:,s),u0(:,s),th,rho(s));
                        z(:,s+1)     = z(:,s) + delta_s(s)*z_prime;              % With FFD (we will need to change this)  
                    end

                    for tt=1:n_states
                        for s=1:N_points
                            if z(tt,s)<lb_z(tt,s) || z(tt,s)>ub_z(tt,s)
                                    flag = 1;   
                            end
                        end
                    end

                    if flag==0
                        return;
                    end
                end
            end
    end
end

