%% BVP Conditions    
    disp('BVP Conditions')
    R_e = 6378140;              % Earth's radius in m
    g_0 = 9.81;                 % Acceleration due to gravity in m/s^2
    mu = 398574405096000;       % Gravitational constant times Earth's mass in m^3/s^2
    m_0 = 6000;                 % Initial mass in kg

    V_e = 50000;                % Thruster Exhaust Velocity in m/s
    V_E = V_e/sqrt(g_0*R_e);    % non-dimensionalized V_E


    m_c = 0.0392;               % Fuel consumption rate in kg/s
    %Mc = m_c/(2*m_0/tf);       % non-dimensional fuel consumption

    % dimensional initial conditions
    R_0 = R_e + 450000;         % Initial geocentric distance in m
    V_0 = sqrt(mu/R_0);         % Initial velocity in m/s
    disp(V_0)

    % non-dimensional initial conditions
    R_0 = R_0/R_e;              % Initial geocentric distance in km
    V_0 = V_0/sqrt(g_0*R_e);    % Initial velocity in m/s
    m_0 = m_0/m_0;              % Initial mass in kg

    gamma_0 = 0;                % Initial flight path angle
    alpha_0 = 0;                % Thrust angle of attack

    %Terminal state constraints
    h_f = 1000000;               % Final altitude in m
    R_f = R_e + h_f;            % Final geocentric distance in m
    V_f = sqrt(mu/R_f);       % Initial velocity in m/s

    % non-dimensional Terminal state constraints
    R_f = R_f/R_e;   % Final geocentric distance in
    V_f = V_f/sqrt(g_0*R_e);
    gamma_f = 0;      % Final flight path angle

    disp('Non dimensionalized Initial Conditions')
    disp('R')
    disp(R_0)
    disp('V')
    disp(V_0)
    disp('gamma')
    disp(gamma_0)

    disp('Non dimensionalized Final Conditions')
    disp('R')
    disp(R_f)
    disp('V')
    disp(V_f)
    disp('gamma')
    disp(gamma_f)

%% Initialization Conditons
    disp('Initialization Conditions')

    R_e = 6378160;       % Earth's radius in m
    g_0 = 9.81;          % Acceleration due to gravity in m/s^2
    mu = 398574405096000;    % Gravitational constant times Earth's mass in m^3/s^2

    % dimensional initial conditions
    R_0 = R_e + 450000;       % Initial geocentric distance in m
    V_0 = sqrt(mu / R_0);  % Initial velocity in m/s
    m_0 = 6000;            % Initial mass in kg

    % non-dimensional initial conditions
    R_0 = R_0/R_e;       % Initial geocentric distance in m
    V_0 = V_0/sqrt(g_0*R_e);  % Initial velocity in m/s
    m_0 = m_0/m_0;            % Initial mass in kg

    gamma_0 = 0;           % Initial flight path angle
    alpha = 0;             % Thrust angle of attack

    disp('Non dimensionalized Initial Conditions')
    disp('R')
    disp(R_0)
    disp('V')
    disp(V_0)
    disp('gamma')
    disp(gamma_0)