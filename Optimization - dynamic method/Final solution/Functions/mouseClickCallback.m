% Define the callback function
function mouseClickCallback(~, ~)
    % Get the current point of the click in the axes coordinates
    pt = get(gca, 'CurrentPoint');
    xClick = pt(1, 1);
    yClick = pt(1, 2);
    
    % Access the variables in the base workspace
    xopt = evalin('base', 'track_new.xopt');
    yopt = evalin('base', 'track_new.yopt');
    vx_full_opt = evalin('base', 'vx_full_opt');
    vy_full_opt = evalin('base', 'vy_full_opt');
    omega_full_opt = evalin('base', 'omega_full_opt');
    n_full_opt = evalin('base', 'n_full_opt');
    epsi_full_opt = evalin('base', 'epsi_full_opt');
    omega_f_full_opt = evalin('base', 'omega_f_full_opt');
    omega_r_full_opt = evalin('base', 'omega_r_full_opt');
    T_drive_opt_full = evalin('base', 'T_drive_opt_full');
    T_brake_opt_full = evalin('base', 'T_brake_opt_full');
    delta_opt_full = evalin('base', 'delta_opt_full');
    s_full         = evalin('base','s_full');
    
    % Find the closest point in the optimal racing line
    distances = sqrt((xopt - xClick).^2 + (yopt - yClick).^2);
    [~, k] = min(distances);
    
    % Get the previous text object handle from appdata
    hText = getappdata(gcf, 'hText');
    
    % Delete the previous text object if it exists
    if ~isempty(hText) && ishandle(hText)
        delete(hText);
    end
    
    % Display the values of the selected variables at the selected point
    textString = sprintf(['$v_{x}$ = %.2f $\\frac{km}{h}$ \n $v_{y}$ = %.2f $\\frac{km}{h}$ \n $\\Omega_{z}$ = %.2f $\\frac{rad}{s}$ \n $n$ = %.2f $m$ \n $\\epsilon$ = %.2f $^{\\circ}$ \n' ...
                          '$\\omega_{f}$ = %.2f $rpm$ \n $\\omega_{r}$ = %.2f $rpm$ \n $T_{drive}$ = %.2f $Nm$ \n $T_{brake}$ = %.2f $Nm$ \n' ...
                          '$\\delta$ = %.2f $^{\\circ}$, \n $s$ = %.2f $Nm$'], ...
                          vx_full_opt(k)*3.6, vy_full_opt(k)*3.6, omega_full_opt(k), n_full_opt(k), ...
                          rad2deg(epsi_full_opt(k)), omega_f_full_opt(k)*60/(2*pi), omega_r_full_opt(k)*60/(2*pi), ...
                          T_drive_opt_full(k), T_brake_opt_full(k), rad2deg(delta_opt_full(k)), s_full(k));
   
    hText = text(xopt(k), yopt(k), textString, 'FontSize', 12, 'BackgroundColor', 'white', 'Interpreter', 'latex');
    
    % Save the new text object handle in appdata
    setappdata(gcf, 'hText', hText);
end
