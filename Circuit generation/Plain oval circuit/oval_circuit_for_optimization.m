% This script aims to create a plain oval circuit to test the optimization
% routine. This script saves a variable "oval_for_optimization" which
% contains very low dimensional vectors representing the points of the
% circuit.

clc;    clear all;  close all;

% Load the original oval circuit
load("oval.mat");

% Extract the inner line
xin = x_inner';
yin = y_inner';

% Extract the outer line
xout = x_outer';
yout = y_outer';

% Create the mid line
xmid=zeros(size(xin));
ymid=zeros(size(yin));
for ii=1:length(xin)
    xmid = (xin+xout)/2;
    ymid = (yin+yout)/2;
end

% Plot the circuit
figure;                                                              
plot(xin, yin,'*','LineWidth',2,'Color',[0 0 0]); hold on;
plot(xout, yout,'*','LineWidth',2,'Color',[0.5 0.5 0.5]); hold on;
plot(xmid, ymid,'*','LineWidth',2,'Color',[1 0 0]); hold off;
title('Oval circuit for optimization purposes','FontSize',20);
xlabel('$x [m]$','Interpreter','LaTex','FontSize',20);
ylabel('$y [m]$','Interpreter','LaTex','FontSize',20);
legend('Inner line','Outer line','Mid line','FontSize',20);
% xlim([0 max(t)]);   
% ylim([0, max(i)]);                                                 
grid on;
set(gca, 'FontSize', 20);

% Save the final variable
save("oval_for_optimization","xin","yin","xout","yout","xmid","ymid");