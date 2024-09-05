clc; clear all; close all;

%% Load a result
% load("Solutions\Simple Curve\solution_simplecurve_dsk_col_30.mat");     % Load the solution on the simple curve
% load("Solutions\Yas Marina\solution_YasMarina_dsk_col_70.mat");           % Load the solution on the Yas Marina circuit with dsk_col=70 m
load("Solutions\Yas Marina\solution_YasMarina_dsk_col_50.mat");           % Load the solution on the Yas Marina circuit with dsk_col=50 m


%% Plot the results

% States
figure;
plot(s_full,vx_full_opt*3.6,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,vx_max*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,vx_min*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(1,:)*z_norm_factor(1)*3.6,'LineWidth',2,'Color',[0 1 0]);
legend('$v_{x,opt}$','$v_{x,max}$','$v_{x,min}$','$v_{x,init}$','interpreter','latex','FontSize',16);
title('Longitudinal speed','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$v_{x} \, [\frac{km}{h}]$','Interpreter','LaTex','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,vy_full_opt*3.6,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,vy_max*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,vy_min*3.6*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(2,:)*z_norm_factor(2)*3.6,'LineWidth',2,'Color',[0 1 0]);
legend('$v_{y,opt}$','$v_{y,max}$','$v_{y,min}$','$v_{y,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$v_{y} \, [\frac{km}{h}]$','Interpreter','LaTex','FontSize',16);
title('Lateral speed','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,omega_full_opt,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,omega_z_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,omega_z_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(3,:)*z_norm_factor(3),'LineWidth',2,'Color',[0 1 0]);
legend('$\Omega_{z,opt}$','$\Omega_{z,max}$','$\Omega_{z,min}$','$\Omega_{z,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\Omega_{z} \, [\frac{rad}{s}]$','Interpreter','LaTex','FontSize',16);
title('Yaw rate','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,n_full_opt,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,n_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,n_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(4,:)*z_norm_factor(4),'LineWidth',2,'Color',[0 1 0]);
legend('$n_{opt}$','$n_{max}$','$n_{min}$','$n_{init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$n [m]$','Interpreter','LaTex','FontSize',16);
title('Transversal displacement','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,rad2deg(epsi_full_opt),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,rad2deg(epsi_max)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,rad2deg(epsi_min)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,rad2deg(z0(5,:)*z_norm_factor(5)),'LineWidth',2,'Color',[0 1 0]);
legend('$\epsilon_{opt}$','$\epsilon_{max}$','$\epsilon_{min}$','$\epsilon_{init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\epsilon \, [deg]$','Interpreter','LaTex','FontSize',16);
title('Heading angle','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,omega_f_full_opt*60/(2*pi),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,omega_f_max*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,omega_f_min*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(6,:)*z_norm_factor(6)*60/(2*pi),'LineWidth',2,'Color',[0 1 0]);
legend('$\Omega_{f,opt}$','$\Omega_{f,max}$','$\Omega_{f,min}$','$\Omega_{f,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\Omega_{f} \, [rpm]$','Interpreter','LaTex','FontSize',16);
title('Front wheel rotational speed','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,omega_r_full_opt*60/(2*pi),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,omega_r_max*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,omega_r_min*60/(2*pi)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,z0(7,:)*z_norm_factor(7)*60/(2*pi),'LineWidth',2,'Color',[0 1 0]);
legend('$\Omega_{r,opt}$','$\Omega_{r,max}$','$\Omega_{r,min}$','$\Omega_{r,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\Omega_{r} \, [rpm]$','Interpreter','LaTex','FontSize',16);
title('Rear wheel rotational speed','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

% Inputs
figure;
plot(s_full,T_drive_opt_full,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,T_drive_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,T_drive_min*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,u0(1,:)*u_norm_factor(1),'LineWidth',2,'Color',[0 1 0]);
legend('$T_{drive,opt}$','$T_{drive,max}$','$T_{drive,min}$','$T_{drive,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$T_{drive} \, [Nm]$','Interpreter','LaTex','FontSize',16);
title('Driving torque','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,T_brake_opt_full,'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,T_brake_max*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_grid,u0(2,:)*u_norm_factor(2),'LineWidth',2,'Color',[0 1 0]);
legend('$T_{brake,opt}$','$T_{brake,max}$','$T_{brake,init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$T_{brake} \, [Nm]$','Interpreter','LaTex','FontSize',16);
title('Braking torque','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,rad2deg(delta_opt_full),'LineWidth',2,'Color',[0 0 1]); hold on;
plot(s_full,rad2deg(delta_max)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle','--'); hold on;
plot(s_full,rad2deg(delta_min)*ones(size(s_full)),'LineWidth',2,'Color',[1 0 0],'LineStyle',':'); hold on;
plot(s_grid,rad2deg(u0(3,:)*u_norm_factor(3)),'LineWidth',2,'Color',[0 1 0]);
legend('$\delta_{opt}$','$\delta_{max}$','$\delta_{min}$','$\delta_{init}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$\delta \, [deg]$','Interpreter','LaTex','FontSize',16)
title('Steering angle','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

figure;
plot(s_full,T_brake_opt_full,'b','LineWidth',2); hold on;
plot(s_full,T_drive_opt_full,'r','LineWidth',2);
legend('$T_{brake,opt}$','$T_{drive,opt}$','interpreter','latex','FontSize',16);
xlabel('$s [m]$','Interpreter','LaTex','FontSize',16);
ylabel('$T \, [Nm]$','Interpreter','LaTex','FontSize',16);
title('Braking torque vs Driving Torque','FontSize',16);
grid on;
set(gca, 'FontSize', 16);

%% Plot optimal racing line with the corresponding state values (point-by-point)
% Here you can click on the figure to see the state values at the desired point

figure;
plot(track_new.Xl(:,1), track_new.Xl(:,2), 'LineWidth', 1,'Color',[0 0 0]); hold on;
plot(track_new.Xr(:,1), track_new.Xr(:,2), 'LineWidth', 1,'Color',[0.5 0.5 0.5]); hold on;
plot(track_new.xopt, track_new.yopt, 'LineWidth', 1.5,'Color',[0 0.5 0]);
plot(track_new.xopt(1), track_new.yopt(1),'.', 'MarkerSize', 15,'Color',[1 0 0]);
legend('Outer line', 'Inner line', 'Optimal racing line','Starting point', 'FontSize', 14);
title('Optimal solution', 'FontSize', 16);
xlabel('$X [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
ylabel('$Y [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
grid on;
set(gca, 'FontSize', 16);
hFig = gcf;
setappdata(hFig, 'hText', []);
set(hFig, 'WindowButtonDownFcn', @mouseClickCallback);

%% Save the solution
% save('Solutions\solution_');

%% Create and save a video with the final solution
numFrames = length(s_full);                                      
frames_vec(numFrames) = struct('cdata', [], 'colormap', []);     % Initialize an array to store the frames

for ii=1:length(s_full)
    hFig = figure(30);
    set(hFig,'WindowState', 'maximized');
    plot(track_new.Xl(:,1), track_new.Xl(:,2), 'LineWidth', 1,'Color',[0 0 0]); hold on;
    plot(track_new.Xr(:,1), track_new.Xr(:,2), 'LineWidth', 1,'Color',[0.5 0.5 0.5]); hold on;
    plot(track_new.xopt, track_new.yopt, 'LineWidth', 1.5,'Color',[0 0.5 0]); hold on;
    plot(track_new.xopt(1), track_new.yopt(1),'.', 'MarkerSize', 15,'Color',[1 0 0]); hold on;
    plot(track_new.xopt(ii), track_new.yopt(ii),'^', 'MarkerSize', 7,'Color',[0 0 1],'MarkerFaceColor','b'); hold off;
    legend('Outer line', 'Inner line', 'Optimal racing line','Starting point','Vehicle position', 'FontSize', 14);
    title(['Optimal lap simulation - current time: ' num2str(round(laptime_vec_opt_full(ii)*100)/100), ' s'], 'FontSize', 16);
    xlabel('$X [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
    ylabel('$Y [m]$', 'Interpreter', 'LaTex', 'FontSize', 16);
    grid on;
    set(gca, 'FontSize', 16);
    textString = sprintf(['$v_{x}$ = %.2f $\\frac{km}{h}$ \n $v_{y}$ = %.2f $\\frac{km}{h}$ \n $\\Omega_{z}$ = %.2f $\\frac{rad}{s}$ \n $n$ = %.2f $m$ \n $\\epsilon$ = %.2f $^{\\circ}$ \n' ...
                          '$\\Omega_{f}$ = %.2f $rpm$ \n $\\Omega_{r}$ = %.2f $rpm$ \n $T_{drive}$ = %.2f $Nm$ \n $T_{brake}$ = %.2f $Nm$ \n' ...
                          '$\\delta$ = %.2f $^{\\circ}$ \n $s$ = %.2f $m$'], ...
                          vx_full_opt(ii)*3.6, vy_full_opt(ii)*3.6, omega_full_opt(ii), n_full_opt(ii), ...
                          rad2deg(epsi_full_opt(ii)), omega_f_full_opt(ii)*60/(2*pi), omega_r_full_opt(ii)*60/(2*pi), ...
                          T_drive_opt_full(ii), T_brake_opt_full(ii), rad2deg(delta_opt_full(ii)), s_full(ii));
    hText = text(track_new.xopt(ii)+10,  track_new.yopt(ii)+10, textString, 'FontSize', 12, 'BackgroundColor', 'white', 'Interpreter', 'latex');
    pause(0.01);
    frames_vec(ii) = getframe(hFig);
    clear hText;
end

% Save the video
hFigVisible = figure;
set(hFigVisible,'WindowState', 'maximized');
set(hFigVisible, 'PaperPositionMode', 'auto');
set(hFigVisible, 'Units', 'normalized');
set(hFigVisible, 'Position', [0, 0, 1, 1]);
set(hFigVisible, 'Visible', 'on');
video_duration = total_lap_time_opt;
frameRate = round(numFrames/video_duration);
movie(hFigVisible, frames_vec, 1, frameRate);