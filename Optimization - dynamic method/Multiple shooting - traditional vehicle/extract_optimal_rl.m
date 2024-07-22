function [x_opt_rl, y_opt_rl] = extract_optimal_rl(xin, yin, xout, yout, xmid, ymid, d)
    if d >= 0
        % Calcola la distanza tra (xmid, ymid) e (xin, yin)
        dist_in = sqrt((xin-xmid)^2 + (yin-ymid)^2);
        % Calcola le coordinate del punto sul segmento che dista d da (xmid, ymid)
        x_opt_rl = xmid + d * (xin - xmid) / dist_in;
        y_opt_rl = ymid + d * (yin - ymid) / dist_in;
    else
        % Calcola la distanza tra (xmid, ymid) e (xout, yout)
        dist_out = sqrt((xout-xmid)^2 + (yout-ymid)^2);
        % Calcola le coordinate del punto sul segmento che dista abs(d) da (xmid, ymid)
        x_opt_rl = xmid + abs(d) * (xout - xmid) / dist_out;
        y_opt_rl = ymid + abs(d) * (yout - ymid) / dist_out;
    end
end
