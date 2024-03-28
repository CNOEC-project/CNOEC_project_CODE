function [delta_A,w] = Static_variable_computation(x_in,y_in,x_out,y_out,wv,e)

N=length(x_in);
w       =   sqrt( (x_out-x_in).^2 + (y_out-y_in).^2 );      % vector of track width [m]
delta_A = (wv/2+e)./w;                                      % vector of delta_alpha

end
