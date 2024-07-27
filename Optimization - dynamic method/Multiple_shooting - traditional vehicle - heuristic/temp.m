clc;
clear all;
close all;
%%

% Specify the interval within which the solution is to be searched
    omega_delta_range = linspace(-5*pi/180,5*pi/180,11);
    Tdr_range = linspace(-100,100,9);
    Tdf_range = linspace(-100,100,9);
    
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