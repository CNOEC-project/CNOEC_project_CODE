function [rho,w,delta_theta,delta_s] = static_variable_computation(x_in,y_in,x_out,y_out,xmid,ymid)

N=length(x_in);
w       =   sqrt( (x_out-x_in).^2 + (y_out-y_in).^2 );      % vector of track width [m]

delta_theta = zeros(N,1);
    for ii=2:N-1
        delta_theta(ii,1)=atan2(ymid(ii+1)-ymid(ii),xmid(ii+1)-xmid(ii))-atan2(ymid(ii)-ymid(ii-1),xmid(ii)-xmid(ii-1));
    end

delta_s = zeros(N,1);
    for ii=1:N-1
        delta_s(ii,1) = euclidean_distance(xmid(ii+1),ymid(ii+1),xmid(ii),ymid(ii));
    end

rho = delta_theta./delta_s;
rho(end,1)=0;
end




% delta_A = (wv/2+e)./w;                                      % vector of delta_alpha