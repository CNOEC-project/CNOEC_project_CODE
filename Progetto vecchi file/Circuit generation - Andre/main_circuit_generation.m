clc
clear all
close all 

%% Import the circuit
[lat,lon,alt] = read_kml("Yas_Marina.kml");
alt = 0;
origin = [lat(1), lon(1), alt];
[x,y,z] = latlon2local(lat,lon,alt,origin);

%% Internal line of the circuit 

xmap=x(1:375,:);
ymap=y(1:375,:);

%% Data cleaning poiché alcuni dati sono campionati male

xmap(310)    =   [];
ymap(310)    =   [];
xmap(309)    =   [];
ymap(309)    =   [];
xmap(307)    =   [];
ymap(307)    =   [];
xmap(364)    =   [];
ymap(364)    =   [];
N=length(xmap);

%% Creazione circuito esterno

% Passo 3: Creazione del vettore di punti pin
pmap = [xmap(:), ymap(:)];  % Assicura che pin sia una matrice con due colonne

% Rimuovi eventuali duplicati
[unique_xin, idx] = unique(xmap);
pin_unique = pmap(idx, :);

% Passo 4: Interpolazione lineare
pin_interp = interp1(unique_xin, pin_unique, xmap, 'linear', 'extrap');

% Passo 5: Calcolo delle rette ortogonali
m = diff(pin_interp(:, 2)) ./ diff(pin_interp(:, 1));  % Calcolo delle pendenze
m_orto = -1./m;  % Calcolo delle pendenze ortogonali
b_orto = pin_interp(1:end-1, 2) - m_orto .* pin_interp(1:end-1, 1);  % Calcolo degli offset

% Passo 6: Selezione dei punti distanti d lungo le rette ortogonali
d = 10;

xout = zeros(N-1, 1);
yout = zeros(N-1, 1);

for i = 1:N-1
    % Calcola il punto ortogonale distante d da pin(i)
    if xmap(i) < xmap(i+1) && ymap(i) < ymap(i+1)
        xout(i) = pmap(i, 1) - d / sqrt(1 + m_orto(i)^2);            
    elseif xmap(i) < xmap(i+1) && ymap(i) > ymap(i+1)
        xout(i) = pmap(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif xmap(i) > xmap(i+1) && ymap(i) > ymap(i+1)
        xout(i) = pmap(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif xmap(i) > xmap(i+1) && ymap(i) < ymap(i+1)
        xout(i) = pmap(i, 1) - d / sqrt(1 + m_orto(i)^2);
    end
    yout(i) = m_orto(i) * xout(i) + b_orto(i);
end

xout(end+1) = xout(1);
yout(end+1) = yout(1);

% Passo 7: Plot dei risultati
figure;
plot(xmap, ymap, 'o-', 'DisplayName', 'Punti originali');
hold on;
plot(pin_interp(:, 1), pin_interp(:, 2), 'r.-', 'DisplayName', 'Interpolazione lineare');
plot(xout, yout,'kx', 'DisplayName', 'Punti ortogonali selezionati');
hold on 
plot(xout, yout, 'DisplayName', 'Punti ortogonali selezionati interpolati');
legend;
xlabel('X');
ylabel('Y');
title('Interpolazione lineare e Punti ortogonali');
grid on;
hold off;

% Sampling fra i punti

N_points                  =       100;
x_matin                   =       zeros(N-1,N_points);
y_matin                   =       zeros(N-1,N_points);

for ii=1:N-1                                                                         % for each interval
        x_matin(ii,:)       =       linspace(xmap(ii),xmap(ii+1),N_points);               % create the x matrix that was previously initialized
        y_matin(ii,:)       =       linspace(ymap(ii),ymap(ii+1),N_points);
end

xin = (reshape(x_matin.', 1, []))';                                                    % Creazione di un unico vettore contenente tutti i punti xin sovracampionati
yin = (reshape(y_matin.', 1, []))';                                                    % Creazione di un unico vettore contenente tutti i punti yin sovracampionati

x_matout                  =       zeros(N-1,N_points);
y_matout                  =       zeros(N-1,N_points);

for ii=1:N-1                                                                               % for each interval
        x_matout(ii,:)       =       linspace(xout(ii),xout(ii+1),N_points);               % create the x matrix that was previously initialized
        y_matout(ii,:)       =       linspace(yout(ii),yout(ii+1),N_points);
end

xout = (reshape(x_matout.', 1, []))';                                                    % Creazione di un unico vettore contenente tutti i punti xout sovracampionati
yout = (reshape(y_matout.', 1, []))';                                                    % Creazione di un unico vettore contenente tutti i punti yout sovracampionati

% Plot dei risultati finali
figure(3)
plot(xin,yin),hold on;
plot(xout,yout),hold on;

%% Salviamo i risultati
save('circuit_points','xin','yin','xout','yout');

%% Definizione delle variabili geometriche necessarie

wv          =       4;                  % vehicle width [m]
e           =       0.1;                % same margin [m]
%% Derivazione delle variabili d'interesse del circuito

[delta_A,w]         =       Static_variable_computation(xin,yin,xout,yout,wv,e);