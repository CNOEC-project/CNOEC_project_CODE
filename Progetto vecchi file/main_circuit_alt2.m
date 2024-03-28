clear all
close all
[lat,lon,~] = read_kml("Yas_Marina.kml");
alt = 0;
origin = [lat(1), lon(1), alt];
[x,y,z] = latlon2local(lat,lon,alt,origin);

% figure();
% plot(x,y),grid on;
% 
% figure();plot(lat,lon),grid on;

%% Divisione tracciato interno esterno

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

% figure();
% plot(x_in_map,y_in_map,'Blue',x_out_map,y_out_map,'red'),grid on;

%calcolo lunghezza tracciato interno/esterno
% J_l=0;
%   for j=2:375
%      J_l=J_l +sqrt((x_in_map(j)-x_in_map(j-1))^2+(y_in_map(j)-y_in_map(j-1))^2);
%   end
%  J_l 
% 
%  J_l2=0;
%   for j=2:148
%      J_l2=J_l2 +sqrt((x_out_map(j)-x_out_map(j-1))^2+(y_out_map(j)-y_out_map(j-1))^2);
%   end
%  J_l2
% viene circa 4.6 km forse è scalato male dato che internet dice 5km???

%% aumento numero campioni da 553 a 54mila tramite interpolazione lineare

alpha=1/length(x_out_map);
for i=1:length(x_in_map)-1
    for j=1:length(x_out_map)-1
        x_in1(j,i)=x_in_map(i)+alpha*j*(x_in_map(i+1)-x_in_map(i));
        y_in1(j,i)=y_in_map(i)+alpha*j*(y_in_map(i+1)-y_in_map(i));

    end
end

beta=1/length(x_in_map);
for i=1:length(x_out_map)-1
    for j=1:length(x_in_map)-1
        x_out1(j,i)=x_out_map(i)+beta*j*(x_out_map(i+1)-x_out_map(i));
        y_out1(j,i)=y_out_map(i)+beta*j*(y_out_map(i+1)-y_out_map(i));

    end
end

x_in3=reshape(x_in1,1,54978);
y_in3=reshape(y_in1,1,54978);
x_out2=reshape(x_out1,1,54978);
y_out2=reshape(y_out1,1,54978);

% scatter(x_in(1),y_in(1)),hold on;
% scatter(x_out(26090),y_out(26090));
%plottato per vedere come posso avere punti vicini come partenza

x_out3=zeros(1,length(x_in3));
y_out3=zeros(1,length(x_in3));

for i=1:length(x_in3)
    if i<=length(x_in3)-26089
      x_out3(i)=x_out2(i+26089);
      y_out3(i)=y_out2(i+26089);
    else
      x_out3(i)=x_out2(i-length(x_in3)+26089);
      y_out3(i)=y_out2(i-length(x_in3)+26089);
    end
end

% figure();
% plot(x_in3,y_in3,'Blue',x_out3,y_out3,'red'),grid on;

% scatter(x_in3,y_in3);
% hold on;
% scatter(x_out3,y_out3);

%% Riduzione campioni di un fattore 100 circa e riallineamento grossolano sul quale rifarò l'interpolazione lineare

h=round(length(x_in3)/100);

%predefinizione dei vettori di output per velocità
x_in4=zeros(1,h);
y_in4=zeros(1,h);
x_out4=zeros(1,h);
y_out4=zeros(1,h);

%traiettoria chiusa
x_in4(1)=x_in3(1);
y_in4(1)=y_in3(1);
x_out4(1)=x_out3(1);
y_out4(1)=y_out3(1);

j=2;
for i=2:h
   while sqrt((x_out3(j+1)-x_out4(i-1))^2+(y_out3(j+1)-y_out4(i-1))^2)<8.5 && j<length(x_out3)-1
       j=j+1;
   end
   x_out4(i)=x_out3(j);
   y_out4(i)=y_out3(j);
end

k=2;
for i=2:h
   while sqrt((x_in3(k+1)-x_in4(i-1))^2+(y_in3(k+1)-y_in4(i-1))^2)<8.5 && k<length(x_out3)-1
       k=k+1;
   end
   x_in4(i)=x_in3(k);
   y_in4(i)=y_in3(k);
end

%rescaling per evitare i valori uguali alla fine

for j=1:515
     if j<515
        x_in5(j)=x_in4(j);
        y_in5(j)=y_in4(j);
     else
         x_in5(j)=x_in4(1);
         y_in5(j)=y_in4(1);
     end
end

for j=1:527
     if j<527
        x_out5(j)=x_out4(j);
        y_out5(j)=y_out4(j);
     else
         x_out5(j)=x_out4(1);
         y_out5(j)=y_out4(1);
     end
end

% figure();
% scatter(x_in5,y_in5);
% hold on;
% scatter(x_out5,y_out5);

%% ora che ho campioni equidistanti con una nuova interpolazione molto più fine dovrebbe funzionare


gamma=1/length(x_out5);
for i=1:length(x_in5)-1
    for j=1:length(x_out5)-1
        x_in6(j,i)=x_in5(i)+gamma*j*(x_in5(i+1)-x_in5(i));
        y_in6(j,i)=y_in5(i)+gamma*j*(y_in5(i+1)-y_in5(i));

    end
end

sigma=1/length(x_in5);
for i=1:length(x_out5)-1
    for j=1:length(x_in5)-1
        x_out6(j,i)=x_out5(i)+sigma*j*(x_out5(i+1)-x_out5(i));
        y_out6(j,i)=y_out5(i)+sigma*j*(y_out5(i+1)-y_out5(i));

    end
end

x_in7=reshape(x_in6,1,270364);
y_in7=reshape(y_in6,1,270364);
x_outa=reshape(x_out6,1,270364);
y_outa=reshape(y_out6,1,270364);

% scatter(x_in7(1),y_in7(1)),hold on;
% scatter(x_outa(13),y_outa(13));
% %plottato per vedere come posso avere punti vicini come partenza

x_out7=zeros(1,length(x_in7));
y_out7=zeros(1,length(x_in7));

for i=1:length(x_in7)
    if i<=length(x_in7)-12
      x_out7(i)=x_outa(i+12);
      y_out7(i)=y_outa(i+12);
    else
      x_out7(i)=x_outa(i-length(x_in7)+12);
      y_out7(i)=y_outa(i-length(x_in7)+12);
    end
end
y_out7(1)=y_in7(1);

% figure();
% plot(x_in7,y_in7,'Blue',x_out7,y_out7,'red'),grid on;

% scatter(x_in7,y_in7);
% hold on;
% scatter(x_out7,y_out7);

%% campionamento finale alleluja

h=1000; %da cambiare per cambiare il numero campioni (1000 son 4 metri circa)

%predefinizione dei vettori di output per velocità
x_outer=zeros(1,h);
y_outer=zeros(1,h);
x_inner=zeros(1,h);
y_inner=zeros(1,h);

%traiettoria chiusa
x_outer(1)=x_out7(1);
y_outer(1)=y_out7(1);
x_inner(1)=x_in7(1);
y_inner(1)=y_in7(1);
x_outer(h)=x_out7(1);
y_outer(h)=y_out7(1);
x_inner(h)=x_in7(1);
y_inner(h)=y_in7(1);

k=2;
for i=2:h-1
   while sqrt((x_in7(k+1)-x_inner(i-1))^2+(y_in7(k+1)-y_inner(i-1))^2)<4.3265 && k<length(x_in7)-1
       k=k+1;
   end
   x_inner(i)=x_in7(k);
   y_inner(i)=y_in7(k);
end

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

x_out = zeros(1,length(x_inner));
y_out = zeros(1,length(x_inner));
x_outer = zeros(1,length(x_inner));
y_outer = zeros(1,length(x_inner));

for i = 1:length(x_inner)-1
    % Calcola il punto ortogonale distante d da pin(i)
    if x_inner(i) < x_inner(i+1) && y_inner(i) < y_inner(i+1)
        x_out(i) = pmap(i, 1) - d / sqrt(1 + m_orto(i)^2);            
    elseif x_inner(i) < x_inner(i+1) && y_inner(i) > y_inner(i+1)
        x_out(i) = pmap(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif x_inner(i) > x_inner(i+1) && y_inner(i) > y_inner(i+1)
        x_out(i) = pmap(i, 1) + d / sqrt(1 + m_orto(i)^2);
    elseif x_inner(i) > x_inner(i+1) && y_inner(i) < y_inner(i+1)
        x_out(i) = pmap(i, 1) - d / sqrt(1 + m_orto(i)^2);
    end
    y_out(i) = m_orto(i) * x_out(i) + b_orto(i);
end

d=zeros(length(x_out7),length(x_inner));
for i=1:length(x_out7)
    for j=1:length(x_inner)
        d(i,j)=sqrt((x_out(j)-x_out7(i))^2+(x_out(j)-x_out7(i))^2);
    end
end 

%%
[M,N]=size(d);
m=zeros(1,N);


for i=1:N
    minimum=min(d(:,i));
    for k=1:M
        if d(k,i)==minimum
           l=k;
        end
    end
    m(i)=l;
end

 for j=1:length(x_inner)-1
     x_outer(j) =x_out7(m(j));
     y_outer(j)=y_out7(m(j)); 
 end

 x_outer(length(x_inner))=x_outer(1);
 x_outer(length(x_inner))=y_outer(1);

% scatter(x_inner,y_inner);
% hold on;
% scatter(x_outer,y_outer);


%% Test mezzeria
alpha=0.5*ones(length(x_inner),1);

for j=1:length(x_inner)
    x_star(j)=x_inner(j)-alpha(j)*(x_inner(j)-x_outer(j));
    y_star(j)=y_inner(j)-alpha(j)*(y_inner(j)-y_outer(j));
end
plot(x_inner,y_inner,'Black',x_outer,y_outer,'Black',x_star,y_star,'Red'),grid on;


