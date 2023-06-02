%% Part 1: Initialization

N = 50;                 % Order of Approximation

[x,D]=legDc(N);
x = flip(x);


R_e = 6378140;              % Earth's radius in m
g_0 = 9.81;                 % Acceleration due to gravity in m/s^2
mu = 398574405096000;       % Gravitational constant times Earth's mass in m^3/s^2
m_0 = 6000;                 % Initial mass in kg

% V_e = 2000;                % Thruster Exhaust Velocity in m/s
% V_E = V_e/sqrt(g_0*R_e);    % non-dimensionalized V_E


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
h_f = 2000000;               % Final altitude in m
R_f = R_e + h_f;            % Final geocentric distance in m
V_f = sqrt(mu/R_f);       % Initial velocity in m/s

% non-dimensional Terminal state constraints
R_f = R_f/R_e;   % Final geocentric distance in
V_f = V_f/sqrt(g_0*R_e);
gamma_f = 0;      % Final flight path angle

% control variable constraints
alpha_min = -pi/2; % Minimum thrust angle of attack
alpha_max = pi/2;  % Maximum thrust angle of attack

tf = 10000;         % initial guess in final time


% Optimization variables and constraints
Z0 = [linspace(1.05,1.16,N+1)'; linspace(0.97,0.92,N+1)'; zeros(N+1,1); zeros(N+1,1); tf];   % Initial guess for optimization variables
% Lb = [1*ones(N+1,1); 0.9*ones(N+1,1); -pi/2*ones(N+1,1); alpha_min*ones(N+1,1); 0];     % Lower bounds of optimization variables
% Ub = [1.2*ones(N+1,1); 1*ones(N+1,1); pi/2*ones(N+1,1); alpha_max*ones(N+1,1); Inf];  % Upper bounds of optimization variables
Lb = [];
Ub = [];

Aeq_init = zeros(4*N+5, 4*N+5);
% boundary conditions conditions
Aeq_init(1,1) = 1;          % R_0
Aeq_init(N+1,N+1) = 1;      % R_f
Aeq_init(N+2,N+2) = 1;      % V_0
Aeq_init(2*N+2,2*N+2) = 1;  % V_f
Aeq_init(2*N+3,2*N+3) = 1;  % gamma_0
Aeq_init(3*N+3,3*N+3) = 1;  % gamma_f
%Aeq_init(3*N+4,3*N+4) = 1;  % alpha_0

% initializing values at boundary conditions
Beq_init = zeros(4*N + 5, 1);
Beq_init(1,1) = R_0;          % R_0
Beq_init(N+1,1) = R_f;      % R_f
Beq_init(N+2,1) = V_0;      % V_0
Beq_init(2*N+2,1) = V_f;  % V_f
Beq_init(2*N+3,1) = gamma_0;  % gamma_0
Beq_init(3*N+3,1) = gamma_f;  % gamma_f
%Beq_init(3*N+4,1) = alpha_0;  % alpha_0



options =  optimoptions ('fmincon','Display','Off','OptimalityTolerance',...
1e-10 , 'ConstraintTolerance' ,1e-5, 'MaxIterations', 2000,'MaxFunctionEvaluations',...
20000, 'Algorithm','sqp-legacy' );
A = [];
b = [];
Aeq = Aeq_init;
Beq = Beq_init;
opt = [];

[Z, costval, exitflag, output] = fmincon(@(Z)costfunc(Z,N), Z0, A, b,Aeq, Beq, Lb, Ub, @(Z)Cfun_s(Z,N),options);

%{
% Display the optimization results
disp('Optimization Results:');
disp(['Cost value: ' num2str(costval)]);
disp(['Exit flag: ' num2str(exitflag)]);
disp(output);
%}

%% Part 2: Plotting
 
% Extract the optimized variables
R_opt = Z(1:N+1);
V_opt = Z(N+2:2*N+2);
gamma_opt = Z(2*N+3:3*N+3);
alpha_opt = Z(3*N+4:4*N+4);
tf_opt = Z(end);

disp(tf_opt)
 
% Calculate the time vector
t = linspace(0, tf_opt, N+1);
 
% Calculate the altitude profile
altitude = (R_opt - 1)*R_e;       % dimensional attitude profile
figure(1);
plot(t, altitude);
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Profile');

% Plot the velocity profile
velocity = V_opt*sqrt(g_0*R_e);
figure(2);
plot(t, velocity);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Velocity Profile');

% Plot the flight path angle profile
figure(3);
plot(t, gamma_opt);
xlabel('Time (s)');
ylabel('Flight Path Angle (rad)');
title('Flight Path Angle Profile');

% Plot the thrust angle profile
figure(4);
plot(t, alpha_opt);
xlabel('Time (s)');
ylabel('Thrust Angle (deg)');
title('Thrust Angle Profile');

%{
% Display the final results
disp('Optimization Results:');
disp(['Cost value: ' num2str(costval)]);
disp(['Exit flag: ' num2str(exitflag)]);
disp(output);
%}
