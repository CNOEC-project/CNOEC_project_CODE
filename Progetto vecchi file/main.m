clear all;
clc;
close all;
[lat,lon,alt] = read_kml("Yas_Marina.kml");
alt = 0;
origin = [lat(1), lon(1), alt];
[x,y,z] = latlon2local(lat,lon,alt,origin);
figure();
plot(x,y),grid on;

figure();plot(lat,lon),grid on;