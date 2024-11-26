function spline = generateSpline(pathToData, varName)
    import casadi.*
    
    rpm2rads = 2*pi/60;
    eps      = 0;

    % Load data for specified varName
    data             = squeeze(ncread(pathToData, varName));

    pitch_data       = deg2rad(ncread(pathToData, 'pitch'))';
    wind_data        = ncread(pathToData, 'wndSpd')';
    omega_data       = ncread(pathToData, 'rotSpd')' * rpm2rads;
  
    % Handle NaN values
    data(isnan(data)) = eps;  % NaNs set to eps

    % Fit spline
    [coeff, betaKnots, windKnots, omegaKnots] = fit3dCxSpline(pitch_data, wind_data, omega_data, data);

    % Create spline
    spline = createParCasadiSpline(varName, {omegaKnots, windKnots, betaKnots}, coeff);
end