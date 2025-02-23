function rho = getDensity(h)
    % Constants
    R = 8.31446;  % Universal gas constant (J/(mol·K))
    M = 0.0289652; % Molar mass of air (kg/mol)
    L = 0.0065;   % Temperature lapse rate in the troposphere (K/m)

    p0 = 101325;  % Ground pressure (Pa) 
    T0 = 288.15;  % Ground temperature (K) 

    % Density calculation using barometric formula
    rho = (p0 * M) / (R * T0) * ((1 - L * h / T0) ^ ((9.8 * M) / (R * L) - 1));
end

