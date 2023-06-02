function [c, ceq, dc, dceq] = Cfun_s(Z,N)
    
    [x, D] = legDc(N);          % constants

    R_e = 6378140;              % Earth's radius in m
    g_0 = 9.81;                 % Acceleration due to gravity in m/s^2
    mu = 398574405096000;       % Gravitational constant times Earth's mass in m^3/s^2
    m_0 = 6000;                 % Initial mass in kg

    V_e =2000;                 % Thruster Exhaust Velocity in m/s
    V_E = V_e/sqrt(g_0*R_e);    % non-dimensionalized V_E


    m_c = 0.0392;               % Fuel consumption rate in kg/s
    %Mc = m_c/(2*m_0/tf);       % non-dimensional fuel consumption

    % dimensional initial conditions
    R_0 = R_e + 450000;         % Initial geocentric distance in m
    V_0 = sqrt(mu/R_0);         % Initial velocity in m/s

    % non-dimensional initial conditions
    R_0 = R_0/R_e;              % Initial geocentric distance in km
    V_0 = V_0/sqrt(g_0*R_e);    % Initial velocity in m/s
    m_0 = m_0/m_0;              % Initial mass in kg

    gamma_0 = 0;                % Initial flight path angle
    alpha_0 = 0;                % Thrust angle of attack

    %Terminal state constraints
    h_f = 2000000;               % Final altitude in m
    R_f = R_e + h_f;            % Final geocentric distance in m
    V_f = sqrt(mu/R_f);       % Initial velocity in m/s

    % non-dimensional Terminal state constraints
    R_f = R_f/R_e;   % Final geocentric distance in
    V_f = V_f/sqrt(g_0*R_e);
    gamma_f = 0;      % Final flight path angle


    R = Z(1:N+1);
    V = Z(N+2:2*N+2);
    gamma = Z(2*N+3:3*N+3);
    alpha  = Z(3*N+4:4*N+4);
    tf = Z(end);

    T_ND = (m_c/(2*m_0/tf)) * V_E;  % non-dimensionalized thrust
    M_ND = (1 - (m_c/(2*m_0/tf)) - (m_c/(2*m_0/tf)).*x);  % non-dimensionalized mass
    k_ND = (tf/2) * sqrt(g_0 / R_e); % non-dimensionalization factor

    % Initialize constraints array
    ceq = zeros(1, 3*(N+1));
    
    % Compute constraint equations
    ceq(1,1:N+1) = (D*R) - ((tf/2)*sqrt(g_0/R_e)) * (V.*sin(gamma));
    ceq(1, ((N+1)+1):((N+1)+(N+1))) = (D * V) - ((T_ND ./ M_ND).* cos(alpha)) + ((k_ND * sin(gamma))./ (R.^2));
    ceq(1, ((2*N+2)+1):((2*N+2) + N+1)) = (D*gamma) - ((T_ND./(M_ND.*V)).*sin(alpha))  + (k_ND* (cos(gamma)./(V.*(R.^2))));

    % ceq(1, (3*N+3)+ 1) = R(N+1) - R_f;
    % ceq(1, (3*N+3)+ 2) = V(N+1) - V_f;
    % ceq(1, (3*N+3)+ 3) = gamma(N+1) - gamma_f;
    % 
    % ceq(1, (3*N+3)+ 4) = R(1) - R_0;
    % ceq(1, (3*N+3)+ 5) = V(1) - V_0;
    % ceq(1, (3*N+3)+ 6) = gamma(1) - gamma_0;
    % ceq(1, (3*N+3)+ 7) = alpha(1) - alpha_0;
    
    
    % Initialize derivatives arrays as empty
    c = [];
    dc = [];
    dceq = [];

end