clc;    clear all;  close all;

%% 
load('circuit_points.mat');
s = cumsum(delta_s);

n = length(s);
indices = []; % Inizializza un array per memorizzare gli indici
current_index = 1; % Fissa il primo indice
indices = [indices, current_index]; % Aggiungi il primo indice alla lista

while current_index < n
    min_diff = inf;
    next_index = current_index;
    for j = current_index+1:n
        difference = abs(abs(s(current_index) - s(j)) - 1);
        if difference < min_diff
            min_diff = difference;
            next_index = j;
        end
    end
    if next_index == current_index
        break; % Se non troviamo un indice migliore, interrompiamo il ciclo
    end
    indices = [indices, next_index]; % Aggiungi il prossimo indice alla lista
    current_index = next_index; % Aggiorna l'indice corrente
end

s_new = s(indices);
max(abs(diff(s_new)));


xin_new = xin(indices(1:end-1));     
yin_new = yin(indices(1:end-1));
xout_new = xout(indices(1:end-1));
yout_new = yout(indices(1:end-1));
x = xmid(indices(1:end-1));
y = ymid(indices(1:end-1));


[k,s] = Static_variable_computation2(x,y);


save('YasMarina_circuit','x','y','k','s');