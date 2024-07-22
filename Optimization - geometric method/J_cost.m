function [out]  = J_cost(alpha, x_inner, y_inner, x_outer, y_outer, k_l, k_rho)

x_i = zeros(size(alpha));
y_i = zeros(size(alpha));
delta_theta = zeros(size(alpha));
delta_s = zeros(size(alpha));

for index_1 = 1:length(x_inner)
    x_i(index_1)= x_inner(index_1)-alpha(index_1)*(x_inner(index_1)-x_outer(index_1));
    y_i(index_1)= y_inner(index_1)-alpha(index_1)*(y_inner(index_1)-y_outer(index_1));
end

%compute length

J_l = 0;

for index_1 = 2:length(x_inner)
    J_l = J_l + (x_i(index_1)-x_i(index_1-1))^2 + (y_i(index_1)-y_i(index_1-1))^2;    
end

J_rho = 0;

for index_1 = 2:length(x_inner)-1
        delta_x_1 = x_i(index_1)-x_i(index_1-1);
        delta_x_2 = x_i(index_1+1)-x_i(index_1);
        delta_y_1 = y_i(index_1)-y_i(index_1-1);
        delta_y_2 = y_i(index_1+1)-y_i(index_1);      
        delta_s(index_1) = sqrt(delta_x_1^2+delta_y_1^2)+sqrt(delta_x_2^2+delta_y_2^2);
        delta_theta(index_1) = atan2(delta_y_2,delta_x_2) - atan2(delta_y_1,delta_x_1); 
end

curvature_vector = (unwrap(delta_theta(2:end-1))./delta_s(2:end-1)).^2;
J_rho = sum(curvature_vector); 

out  = k_l*J_l + k_rho*J_rho;



