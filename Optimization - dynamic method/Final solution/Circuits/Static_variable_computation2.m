function [rho,s] = Static_variable_computation2(xmid,ymid)

    N=length(xmid);
    
    delta_theta = zeros(N,1);
        for ii=2:N-1
            delta_theta(ii,1)=atan2(ymid(ii+1)-ymid(ii),xmid(ii+1)-xmid(ii))-atan2(ymid(ii)-ymid(ii-1),xmid(ii)-xmid(ii-1));
        end
    
    delta_s = zeros(N,1);
        for ii=1:N-1
            delta_s(ii,1) = euclidean_distance(xmid(ii+1),ymid(ii+1),xmid(ii),ymid(ii));
        end
    
    delta_s(N) = euclidean_distance(xmid(N),ymid(N),xmid(1),ymid(1));
    s = cumsum(delta_s);

    rho = delta_theta./delta_s;
    rho(end,1)=0;
end