clc
clear all
close all 

%% Import the circuit
[lat,lon,alt] = read_kml("Yas_Marina.kml");
alt = 0;
origin = [lat(1), lon(1), alt];
[x,y,z] = latlon2local(lat,lon,alt,origin);



%% Creazione circuito interno dai dati...
%Passo 1: separazione punti

k=round(length(x)/2);

for i=1:length(x)
    if i<=k+113
        x_in_map(i)=x(i);
        y_in_map(i)=y(i);
    else 
        x_out_map(i-k-113)=x(i);
        y_out_map(i-k-113)=y(i);
    end
end

%Passo 2:aumento numero campioni da 553 a 54mila tramite interpolazione lineare

beta=1/length(x_in_map);
for i=1:length(x_out_map)-1
    for j=1:length(x_in_map)-1
        x_in1(j,i)=x_out_map(i)+beta*j*(x_out_map(i+1)-x_out_map(i));
        y_in1(j,i)=y_out_map(i)+beta*j*(y_out_map(i+1)-y_out_map(i));

    end
end

x_in2=reshape(x_in1,1,54978);
y_in2=reshape(y_in1,1,54978);

%Passo 3: Riduzione campioni di un fattore 100 circa e riallineamento grossolano sul quale rifarò l'interpolazione lineare

h=round(length(x_in2)/100);

%predefinizione dei vettori di output per velocità
x_in3=zeros(1,h);
y_in3=zeros(1,h);

%traiettoria chiusa
x_in3(1)=x_in2(1);
y_in3(1)=y_in2(1);

j=2;
for i=2:h
   while sqrt((x_in2(j+1)-x_in3(i-1))^2+(y_in2(j+1)-y_in3(i-1))^2)<8.5 && j<length(x_in2)-1
       j=j+1;
   end
   x_in3(i)=x_in2(j);
   y_in3(i)=y_in2(j);
end

%rescaling per evitare i valori uguali alla fine
for j=1:527
     if j<527
        x_in4(j)=x_in3(j);
        y_in4(j)=y_in3(j);
     else
         x_in4(j)=x_in3(1);
         y_in4(j)=y_in3(1);
     end
end

sigma=1/515;
for i=1:length(x_in4)-1
    for j=1:514
        x_in5(j,i)=x_in4(i)+sigma*j*(x_in4(i+1)-x_in4(i));
        y_in5(j,i)=y_in4(i)+sigma*j*(y_in4(i+1)-y_in4(i));

    end
end

x_in=reshape(x_in5,1,270364);
y_in=reshape(y_in5,1,270364);

%Passo 4: campionamento finale traiettoria interna

h=1000; %da cambiare per cambiare il numero campioni (1000 son 4 metri circa)

%predefinizione dei vettori di output per velocità
x_inner=zeros(1,h);
y_inner=zeros(1,h);

%traiettoria chiusa
x_inner(1)=x_in(1);
y_inner(1)=y_in(1);
x_inner(h)=x_in(1);
y_inner(h)=y_in(1);

j=2;
for i=2:h-1
   while sqrt((x_in(j+1)-x_inner(i-1))^2+(y_in(j+1)-y_inner(i-1))^2)<4.43 && j<length(x_in)-1
       j=j+1;
   end
   x_inner(i)=x_in(j);
   y_inner(i)=y_in(j);
end

% scatter(x_inner,y_inner);


%% Creazione circuito esterno....

% Creazione del vettore di punti pin
pmap = [x_inner(:), y_inner(:)];  % Assicura che pin sia una matrice con due colonne

% Passo 1: Calcolo delle rette ortogonali al circuito 

m = diff(y_inner) ./ diff(x_inner);  % Calcolo delle pendenze
m_orto = -1./m;  % Calcolo delle pendenze ortogonali
b_orto=zeros(1,length(x_inner)-1);
for i=1:length(x_inner)-1
    b_orto (i) = y_inner(i) - m_orto(i)* x_inner(i);  % Calcolo degli offset per ogni i
end

% Passo 6: Selezione dei punti distanti d lungo le rette ortogonali
d = 10;

x_outer = zeros(1,length(x_inner));
y_outer = zeros(1,length(x_inner));

for i = 1:length(x_inner)-1
    % Calcola il punto ortogonale distante d da pin(i)
    if x_inner(i) < x_inner(i+1) && y_inner(i) < y_inner(i+1)
        x_outer(i) = pmap(i, 1) - d / sqrt(1 + m_orto(i)^2);            
    elseif x_inner(i) < x_inner(i+1) && y_inner(i) > y_inner(i+1)
        x_outer(i) = pmap(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif x_inner(i) > x_inner(i+1) && y_inner(i) > y_inner(i+1)
        x_outer(i) = pmap(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif x_inner(i) > x_inner(i+1) && y_inner(i) < y_inner(i+1)
        x_outer(i) = pmap(i, 1) - d / sqrt(1 + m_orto(i)^2);
    end
    y_outer(i) = m_orto(i) * x_outer(i) + b_orto(i);
end

x_outer(length(x_inner)) = x_outer(1);
y_outer(length(x_inner)) = y_outer(1);

scatter(x_inner,y_inner);
hold on;
scatter(x_outer,y_outer);



% %% Test mezzeria
% alpha=0.5*ones(length(x_inner),1);
% 
% for j=1:length(x_inner)
%     x_star(j)=x_inner(j)-alpha(j)*(x_inner(j)-x_outer(j));
%     y_star(j)=y_inner(j)-alpha(j)*(y_inner(j)-y_outer(j));
% end
% plot(x_inner,y_inner,'Black',x_outer,y_outer,'Black',x_star,y_star,'Red'),grid on;


