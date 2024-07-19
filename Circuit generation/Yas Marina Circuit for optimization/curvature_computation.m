function [rho,delta_theta,delta_s] = curvature_computation(x_in,y_in)

N=length(x_in);

delta_theta = zeros(N,1);
    for ii=2:N-1
        delta_theta(ii,1)=atan2(y_in(ii+1)-y_in(ii),x_in(ii+1)-x_in(ii))-atan2(y_in(ii)-y_in(ii-1),x_in(ii)-x_in(ii-1));
    end

delta_s = zeros(N,1);
    for ii=1:N-1
        delta_s(ii,1) = euclidean_distance(x_in(ii+1),y_in(ii+1),x_in(ii),y_in(ii));
    end

rho = unwrap(delta_theta./delta_s);
rho(end,1)=0;
end




% delta_A = (wv/2+e)./w;                                      % vector of delta_alpha