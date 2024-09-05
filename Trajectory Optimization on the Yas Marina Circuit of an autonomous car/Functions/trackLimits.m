function [Xl, Xr] = trackLimits(x0,y0,w)
%TRACKLIMITS - Calculate Left and Right Track Limits from Center Line
%
% Inputs:
%   x0 - Vector of x-coordinates of the track center line
%   y0 - Vector of y-coordinates of the track center line
%   w  - Vector or scalar defining the width of the track at each point
%
% Outputs:
%   Xl - Matrix of Cartesian coordinates for the left track limit
%   Xr - Matrix of Cartesian coordinates for the right track limit

X0 = [x0(:) y0(:)];
dx = diff(X0);

n_vec = [-dx(:,2) dx(:,1)]./vecnorm(dx')';
Xl = X0(1:end-1,:) + abs(w(:)/2).*n_vec;
Xr = X0(1:end-1,:) - abs(w(:)/2).*n_vec;

Xl(end+1,:) = 2*Xl(end,:)-Xl(end-1,:);
Xr(end+1,:) = 2*Xr(end,:)-Xr(end-1,:);

end