clear all;
close all;
clc;

%% IMPORTAZIONE DEL FILE TESTO
M = csvread('YasMarina.csv',1,0) 
size = length(M)
track = zeros(size,2);
for i = 1: size
    vect   = [M(i,1), M(i,2)];
    norm = null(vect);
    right  = norm*M(i,3);
    right = right'
    track(i,1) = right(1,1);
    track(i,2) = right(1,2);
end
plot(track(:,1), track(:,2))

