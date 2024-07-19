clc;    clear all;      close all;

%% Load the original Yas Marina Circuit
load('circuit_points');

% Plot the original Yas Marina Circuit
figure(3)
plot(xin,yin,'*','Color',[0.5 0.5 0.5]); hold on;
plot(xout,yout,'*','Color',[0.1 0.3 0.9]); hold on;
grid on;
legend('Inner line','Outer line');
title('Punti del circuito non equi-distanziati');

%% New circuit creation
% The current section undersamples the inner line of original circuit. 
% For the straight lines, we have taken 1 point every 10 meters (circa), except for
% the two longest straight lines, where we have taken 1 point every 20
% meters.
% For the curves, we have taken 1 point every 2 meters (circa).

clear xin_new;
clear yin_new;
xin_new = xin(1);
yin_new = yin(1);

% 1st straight line
[~, idx_x_sup] = min(abs(xin-33.2045));

[~, idx_y_sup] = min(abs(yin-434.912));

idx_x_inf = 1;
idx_y_inf = 1;
point_distance_des = 20;

ii=1;
jj=1;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 1st curve
[~, idx_x_sup] = min(abs(xin-57.0799));

[~, idx_y_sup] = min(abs(yin-457.262));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 2nd straight line
[~, idx_x_sup] = min(abs(xin-159.76));

[~, idx_y_sup] = min(abs(yin-411.464));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;


% 2nd curve
[~, idx_x_sup] = min(abs(xin-178.682));

[~, idx_y_sup] = min(abs(yin-375.345));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 3rd straight
[~, idx_x_sup] = min(abs(xin-245.464));

[~, idx_y_sup] = min(abs(yin-201.858));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 3rd curve
[~, idx_x_sup] = min(abs(xin-381.354));

[~, idx_y_sup] = min(abs(yin-234.706));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 4th straight
[~, idx_x_sup] = min(abs(xin-554.412));

[~, idx_y_sup] = min(abs(yin-188.705));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 4th curve
[~, idx_x_sup] = min(abs(xin-519.536));

[~, idx_y_sup] = min(abs(yin-114.574));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 5th straight
[~, idx_x_sup] = min(abs(xin-(-67.6917)));

[~, idx_y_sup] = min(abs(yin-(-243.269)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 20;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 5th curve
[~, idx_x_sup] = min(abs(xin-(-86.1499)));

[~, idx_y_sup] = min(abs(yin-(-244.875)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 6th straight
[~, idx_x_sup] = min(abs(xin-(-83.0321)));

[~, idx_y_sup] = min(abs(yin-(-196.096)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 6th curve
[~, idx_x_sup] = min(abs(xin-(-92.4868)));

[~, idx_y_sup] = min(abs(yin-(-175.402)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 7th straight
[~, idx_x_sup] = min(abs(xin-(-426.988)));

[~, idx_y_sup] = min(abs(yin-542.127));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 7th curve
[~, idx_x_sup] = min(abs(xin-(-344.3)));

[~, idx_y_sup] = min(abs(yin-544.521));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 8th straight
[~, idx_x_sup] = min(abs(xin-(-336.303)));

[~, idx_y_sup] = min(abs(yin-346.188));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 8th curve
[~, idx_x_sup] = min(abs(xin-(-330.594)));

[~, idx_y_sup] = min(abs(yin-299.112));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 9th straight
[~, idx_x_sup] = min(abs(xin-(-299.299)));

[~, idx_y_sup] = min(abs(yin-206.356));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;


% 9th curve
[~, idx_x_sup] = min(abs(xin-(-277.208)));

[~, idx_y_sup] = min(abs(yin-175.118));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 10th straight
[~, idx_x_sup] = min(abs(xin-(-221.013)));

[~, idx_y_sup] = min(abs(yin-151.166));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 10th curve
[~, idx_x_sup] = min(abs(xin-(-203.818)));

[~, idx_y_sup] = min(abs(yin-170.098));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 11th straight
[~, idx_x_sup] = min(abs(xin-(-197.66)));

[~, idx_y_sup] = min(abs(yin-252.651));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 11th curve
[~, idx_x_sup] = min(abs(xin-(-178.874)));

[~, idx_y_sup] = min(abs(yin-280.893));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 12th straight
[~, idx_x_sup] = min(abs(xin-(-142.482)));

[~, idx_y_sup] = min(abs(yin-281.154));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 12th curve
[~, idx_x_sup] = min(abs(xin-(-127.444)));

[~, idx_y_sup] = min(abs(yin-223.289));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 13th straight
[~, idx_x_sup] = min(abs(xin-(-141.947)));

[~, idx_y_sup] = min(abs(yin-5.363821));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 13th curve
[~, idx_x_sup] = min(abs(xin-(-128.161)));

[~, idx_y_sup] = min(abs(yin-(-66.7936)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 14th straight
[~, idx_x_sup] = min(abs(xin-(-51.4004)));

[~, idx_y_sup] = min(abs(yin-(-144.763)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 10;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 14th curve
[~, idx_x_sup] = min(abs(xin-(-9.31398)));

[~, idx_y_sup] = min(abs(yin-(-114.85)));

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 2;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

% 15th straigth
idx_x_sup = length(xin);

idx_y_sup = length(yin);

idx_x_inf = ii;
idx_y_inf = ii;
point_distance_des = 20;

while ii>=idx_x_inf && ii<=idx_x_sup && ii>=idx_y_inf && ii<=idx_y_sup
    if euclidean_distance(xin(ii),yin(ii),xin_new(jj),yin_new(jj))>=point_distance_des || ii==idx_x_sup
        xin_new(jj+1) = xin(ii);
        yin_new(jj+1) = yin(ii);
        jj=jj+1;
    end
    ii=ii+1;
end
ii=ii-1;
jj=jj-1;

%% Outer line
% The outer line is found by taking a point dis

pin = [xin_new', yin_new'];
N_pin = length(xin_new);

% Calcolo delle rette ortogonali
m = diff(pin(:, 2)) ./ diff(pin(:, 1));  % Calcolo delle pendenze
m_orto = -1./m;  % Calcolo delle pendenze ortogonali
b_orto = pin(1:end-1, 2) - m_orto .* pin(1:end-1, 1);  % Calcolo degli offset

% Passo 6: Selezione dei punti distanti d lungo le rette ortogonali
d = 10;

xout_new = zeros(N_pin-1, 1);
yout_new = zeros(N_pin-1, 1);

for i = 1:N_pin-1
    % Calcola il punto ortogonale distante d da pin(i)
    if xin_new(i) < xin_new(i+1) && yin_new(i) < yin_new(i+1)
        xout_new(i) = pin(i, 1) - d / sqrt(1 + m_orto(i)^2);            
    elseif xin_new(i) < xin_new(i+1) && yin_new(i) > yin_new(i+1)
        xout_new(i) = pin(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif xin_new(i) > xin_new(i+1) && yin_new(i) > yin_new(i+1)
        xout_new(i) = pin(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif xin_new(i) > xin_new(i+1) && yin_new(i) < yin_new(i+1)
        xout_new(i) = pin(i, 1) - d / sqrt(1 + m_orto(i)^2);
    end
    yout_new(i) = m_orto(i) * xout_new(i) + b_orto(i);
end

xout_new(end+1) = xout_new(1);
yout_new(end+1) = yout_new(1);

%% Plot dei risultati finali
figure(3)
plot(xin_new,yin_new,'*','Color',[0.1 0.3 0.9]);  hold on;
plot(xout_new,yout_new,'*','Color',[0.5 0.5 0.5]); 
legend('Inner line','Outer line');
title('Punti del circuito non equi-distanziati');% Plot dei risultati finali

%% Cleaning del circuito
[xin_new_new,yin_new_new,xout_new_new,yout_new_new] = circuit_final_cleaning(xin_new,yin_new,xout_new,yout_new);
[~,idx_rm1] = min(abs(xin_new_new-(-207.995)));

xin_new_new = [xin_new_new(1:idx_rm1-1,1);  xin_new_new(idx_rm1+1:end,1)];
xout_new_new = [xout_new_new(1:idx_rm1-1,1); xout_new_new(idx_rm1+1:end,1)];

yin_new_new = [yin_new_new(1:idx_rm1-1,1);  yin_new_new(idx_rm1+1:end,1)];
yout_new_new = [yout_new_new(1:idx_rm1-1,1); yout_new_new(idx_rm1+1:end,1)];

[~,idx_rm2] = min(abs(xin_new_new-(-207.017)));

xin_new_new = [xin_new_new(1:idx_rm2-1,1);  xin_new_new(idx_rm2+1:end,1)];
xout_new_new = [xout_new_new(1:idx_rm2-1,1); xout_new_new(idx_rm2+1:end,1)];

yin_new_new = [yin_new_new(1:idx_rm2-1,1);  yin_new_new(idx_rm2+1:end,1)];
yout_new_new = [yout_new_new(1:idx_rm2-1,1); yout_new_new(idx_rm2+1:end,1)];

%% Plot dei risultati finali
figure(4)
% plot(xin,yin,'*','Color',[0.5 0.5 0.5]); hold on;
plot(xin_new_new,yin_new_new,'*','Color',[0.1 0.3 0.9]);  hold on;
plot(xout_new_new,yout_new_new,'*','Color',[0.5 0.5 0.5]); 
% plot(xmid,ymid,'*','Color',[0.9 0 0.2]); 
legend('Inner line','Outer line');
title('Punti del circuito non equi-distanziati');% Plot dei risultati finali


%% Plot delle sezioni del circuito
n_oversamp = 100;
x_amp = zeros(1,n_oversamp*length(xin_new_new));
y_amp = zeros(1,n_oversamp*length(xin_new_new));
kk=1;

for ii=1:length(xin_new_new)
    if abs((xout_new_new(ii)-xin_new_new(ii)))>1e-3
        m = (yout_new_new(ii)-yin_new_new(ii))/(xout_new_new(ii)-xin_new_new(ii));
        x_amp(1,kk:kk+n_oversamp-1) = linspace(xin_new_new(ii),xout_new_new(ii),n_oversamp);
        y_amp(1,kk:kk+n_oversamp-1) = m*(x_amp(1,kk:kk+n_oversamp-1)-xin_new_new(ii))+yin_new_new(ii);
        
    else
        y_amp(1,kk:kk+n_oversamp-1) = linspace(yin_new_new(ii),yout_new_new(ii),n_oversamp);
    end
    kk=kk+n_oversamp;
end

figure(5)
plot(xin_new_new,yin_new_new,'*','Color',[0.1 0.3 0.9]);  hold on;
plot(xout_new_new,yout_new_new,'*','Color',[0.5 0.5 0.5]); hold on;
plot(x_amp,y_amp,'*','Color',[1 0 0]); 
legend('Inner line','Outer line','Links');
title('Punti del circuito non equi-distanziati');% Plot dei risultati finali

%% Save the results
xin = xin_new_new;
yin = yin_new_new;
xout = xout_new_new;
yout = yout_new_new;
for ii=1:length(xin)
    xmid = (xin+xout)/2;
    ymid = (yin+yout)/2;
end
% save('circuit_points_optimization','xin','yin','xout','yout','xmid','ymid');



clear xin;
clear yin;
clear xout;
clear yout;
clear xmid;
clear ymid;