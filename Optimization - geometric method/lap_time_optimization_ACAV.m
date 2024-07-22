clear all
clc
close all

%% Yas Marina Circuit
load circuit_points_optimization.mat;
x_inner=xin;
y_inner=yin;
x_outer=xout;
y_outer=yout;

w_i = 10; %track width
w_v = 1.2; %vehicle width

%% Oval circuit
% load oval.mat;
% 
% w_i = 7; %track width
% w_v = 1.2; %vehicle width

%% Rest of the code
N = length(x_inner);

delta_alpha= (w_v+0.03)/w_i;

lb = delta_alpha*ones(N,1);
ub = (1-delta_alpha)*ones(N,1);
alpha_0 =  0.5*ones(N,1);

Ac=zeros(1,N);
Ac(1) = 1;
Ac(end) = -1;
bc = 0;

options = optimoptions('fmincon','MaxFunctionEvaluations',3e6,'MaxIterations',1000,'OptimalityTolerance', 1.0000e-16);

k_rho_v = [0,1];
k_l_v = [1,0];

for index_1 = 1:length(k_rho_v) 
    k_rho = k_rho_v(index_1);
    k_l = k_l_v(index_1);
    alpha_star(:,index_1) = fmincon(@(alpha)J_cost(alpha, x_inner, y_inner, x_outer, y_outer, k_l, k_rho),alpha_0,[],[],Ac,bc,lb,ub,[],options);
end

for index_1 = 1:length(k_rho_v)
    for index_2 = 1:N
        x_star(index_2,index_1)= x_inner(index_2)-alpha_star(index_2,index_1)*(x_inner(index_2)-x_outer(index_2));
        y_star(index_2,index_1)= y_inner(index_2)-alpha_star(index_2,index_1)*(y_inner(index_2)-y_outer(index_2));
    end
end

figure
plot(x_inner, y_inner,'k','LineWidth',1)
hold all
plot(x_outer, y_outer,'k','linewidth',1)

for index_1 = 1:length(k_rho_v)
    plot(x_star(:,index_1), y_star(:,index_1),'linewidth',2)
end
xlabel('x [m]');
ylabel('y [m]');
legend('','','length optimized','curvature optimized')

return
%% mixed

k_rho_v = [0,1,1];
k_l_v = [1,0,1];

for index_1 = 1:length(k_rho_v) 
    k_rho = k_rho_v(index_1);
    k_l = k_l_v(index_1);
    alpha_star(:,index_1) = fmincon(@(alpha)J_cost(alpha, x_inner, y_inner, x_outer, y_outer, k_l, k_rho),alpha_0,[],[],Ac,bc,lb,ub,[],options);
    cost_v(index_1) = J_cost(alpha_star(:,index_1), x_inner, y_inner, x_outer, y_outer, k_l, k_rho);
end

for index_1 = 1:length(k_rho_v)
    for index_2 = 1:N
        x_star(index_2,index_1)= x_inner(index_2)-alpha_star(index_2,index_1)*(x_inner(index_2)-x_outer(index_2));
        y_star(index_2,index_1)= y_inner(index_2)-alpha_star(index_2,index_1)*(y_inner(index_2)-y_outer(index_2));
    end
end

figure
plot(x_inner, y_inner,'k','LineWidth',1)
hold all
plot(x_outer, y_outer,'k','linewidth',1)

for index_1 = 1:length(k_rho_v)
    plot(x_star(:,index_1), y_star(:,index_1),'linewidth',2)
end
xlabel('x [m]');
ylabel('y [m]');
legend('','','length optimized','curvature optimized','mixed')

cost_v(1)
cost_v(2)

%% mixed

options = optimoptions('fmincon','MaxFunctionEvaluations',3e6,'MaxIterations',10000,'OptimalityTolerance', 1.0000e-18,'StepTolerance', 1e-20);

weight_ratio = [0,0.45e-6,0.7e-6,0.8e-6,1e-6,1.2e-6,1.5e-6,10];

for index_1 = 1:length(weight_ratio)
    k_rho = 1;
    k_l = weight_ratio(index_1);
    alpha_star(:,index_1) = fmincon(@(alpha)J_cost(alpha, x_inner, y_inner, x_outer, y_outer, k_l, k_rho),alpha_0,[],[],Ac,bc,lb,ub,[],options);
    cost_v(index_1) = J_cost(alpha_star(:,index_1), x_inner, y_inner, x_outer, y_outer, k_l, k_rho);
end

for index_1 = 1:length(weight_ratio)
    for index_2 = 1:N
        x_star(index_2,index_1)= x_inner(index_2)-alpha_star(index_2,index_1)*(x_inner(index_2)-x_outer(index_2));
        y_star(index_2,index_1)= y_inner(index_2)-alpha_star(index_2,index_1)*(y_inner(index_2)-y_outer(index_2));
    end
    for index_2 = 2:length(x_inner)-1
        delta_x_1 = x_star(index_2,index_1)-x_star(index_2-1,index_1);
        delta_x_2 = x_star(index_2+1,index_1)-x_star(index_2,index_1);
        delta_y_1 = y_star(index_2,index_1)-y_star(index_2-1,index_1);
        delta_y_2 = y_star(index_2+1,index_1)-y_star(index_2,index_1);

        theta_2(index_2,index_1) =  atan(delta_y_2/delta_x_2);
        theta_1(index_2,index_1) =  atan(delta_y_1/delta_x_1);

        delta_s(index_2,index_1) = sqrt(delta_x_1^2+delta_y_1^2)+sqrt(delta_x_2^2+delta_y_2^2);
        delta_theta(index_2,index_1) = atan2(delta_y_2,delta_x_2) - atan2(delta_y_1,delta_x_1);
    end
    curvature_vector(:,index_1) = (unwrap(delta_theta(2:end,index_1))./delta_s(2:end,index_1));
end

%% 
figure
plot(x_inner, y_inner,'k','LineWidth',1)
hold all
plot(x_outer, y_outer,'k','linewidth',1)

for index_1 = 1:length(weight_ratio)
    plot(x_star(:,index_1), y_star(:,index_1),'linewidth',2)
end
xlabel('x [m]');
ylabel('y [m]');

axis equal

figure
subplot(2,1,1)
hold all
for index_1 = 1:length(weight_ratio)
    plot(s, alpha_star(:,index_1),'linewidth',2)
end
box on
xlabel('distance [m]');
ylabel('offset [m]');

subplot(2,1,2)
hold all
for index_1 = 1:length(weight_ratio)
    plot(s(2:end-1), curvature_vector(:,index_1),'linewidth',2)
end
box on
xlabel('distance [m]');
ylabel('curvature [1/m]');


cost_v(1)
cost_v(2)



