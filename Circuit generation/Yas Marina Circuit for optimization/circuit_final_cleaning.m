function [xin_new,yin_new,xout_new,yout_new] = circuit_final_cleaning(xin,yin,xout,yout)
    N_points = length(xin);
    xin_initialized = 1e9*ones(N_points,1);
    yin_initialized = 1e9*ones(N_points,1);
    xout_initialized = 1e9*ones(N_points,1);
    yout_initialized = 1e9*ones(N_points,1);
    kk=1;
    for ii=1:N_points
        flag=0;
        
        for jj=ii+1:N_points
            if checkLineSegmentsIntersection(xin(ii), yin(ii), xout(ii), yout(ii), xin(jj), yin(jj), xout(jj), yout(jj))
                flag=1;
            end
        end

        if flag==0
            xin_initialized(kk)=xin(ii);
            yin_initialized(kk)=yin(ii);
            xout_initialized(kk)=xout(ii);
            yout_initialized(kk)=yout(ii);
            kk=kk+1;
        end
    end
    xin_new = xin_initialized(xin_initialized ~= 1e9);
    yin_new = yin_initialized(yin_initialized ~= 1e9);
    xout_new = xout_initialized(xout_initialized ~= 1e9);
    yout_new = yout_initialized(yout_initialized ~= 1e9);
end