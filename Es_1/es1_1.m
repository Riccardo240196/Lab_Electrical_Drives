clear all; close all; clc; 

%%  Data 

Pn = 75 * 1e3;          % [W]        electric motor power
Jtot = 28;              % [kgm^2]    overall inertia
Nn = 1300;              % [giri/min] motor speed
Q_lm = 1250;            % [l/min]    intake air flow
p_out = 10;             % [bar]      rated pressure
p_in = 1;               % [bar]      ambient pressure
Dp = 1;                 % [bar]      pressure fluctuation
T1 = 20 + 273.15;       % [K]        ambient temperature
Cls = 250;              % [l]        air tank volume
eta_c = 0.6;            % [-]        overall efiiciency
rho1 = 1.245;           % [kg / m^3] air density @ 20°C
cost = 0.13;            % [€ / kWh]  energy cost
k = 1.4;                % k = cp/cv  polytropic process constant
a = [0.25; 0.5; 0.75];  % outlet air flow percentage

%% Units conversion and constants introduction

wn = Nn * 2*pi / 60;    % [rad / s]
Qlm = Q_lm  * 1e-3/ 60; % [m^3 / s]
p_out = p_out * 1e5;    % [Pa]
p_in = p_in * 1e5;      % [Pa]
Dp = Dp * 1e5;          % [Pa]
Cls = Cls * 1e-3;       % [m^3]
R = 287;                % [J/kg/K]   gas constant

Q = zeros(3, 1);
for i = 1 : length(a)
    Q(i) = a(i) * Qlm;  % [l / min]
end

%% Hysteresis on/off control
% First, considering the transformation polytropic is possible to calculate
% the work done by the compressor. Then, with the proper dynamical model of
% the tank we evaluate the capacity and consequently the time related to ON
% and OFF phase. Once the electric power consumed by the compressor is 
% known is possible to find the required torque. At last, after evaluating
% the start time the overall power consumption is found.

% the work done in the compressor is: (p*v^k = const.)
W12 = -(p_in / (rho1 * (k -1))) * (1 - (p_out / p_in)^((k-1) / k));

% the dynamical model of the tank is G1 - G2 = C * dp/dt (like an
% electrical capacitor) the capacity C is calculated as follows:
C = Cls / (R*T1);
G1 = Qlm * rho1; % inlet mass flow rate
G2 = rho1 * Q;   % outlet mass flow rate

% considering the charging condition I can calculate the t_on
t_on = C * 2*Dp ./ (G1 .* (1 - a));

% considering the discharging condition
t_off = C .* 2*Dp ./ (a .* G1);

% the electric power consumed by the compressor (i.e. the motor) is:
Pc = W12 * G1 / eta_c;
Tc = Pc / wn; % the corresponding torque

% from the dynamic equation of the motor: Jtot * wdot = Tm - Tr (Tr = Tc)
Tm = Pn / wn;
t_start = Jtot * wn / (Tm - Tc); % time needed to reach the steady speed 

% so the motor works at nominal power to reach as fast as possible the
% speed and then it works with Pc, thus the total energy is
W_cycle = 0.5 * Pn * t_start + Pc * t_on;

% finally it's possible to calculate the energy consumptionof 1 hour
W_hour_1 = W_cycle .* 3600./(t_start + t_on + t_off);

figure()
bar(a * 100, W_hour_1, 0.5)
grid on;
ylabel('Energy per hour [Wh]', 'interpreter', 'latex');
xlabel('Air flow percentage [\%]', 'interpreter', 'latex')
title('Hysteresis on/off control and constant speed', 'interpreter', 'latex')

%% Speed control
% in this case the compressor always works so we constantly have the
% required pressure and we don't need a tank to store it, hence the power
% after the first start remains constant at Pc. The mass flow rate
% corresponds to the outlet mass flow rate


% the work done in the compressor is: (p*v^k = const.)
W12 = -(p_in / (rho1 * (k -1))) * (1 - (p_out / p_in)^((k-1) / k));

% the electric power consumed by the compressor (i.e. the motor) is:
Pc = W12 * G2 ./ eta_c;
Tc = Pc ./ wn; % the corresponding torque

% from the dynamic equation of the motor: Jtot * wdot = Tm - Tr (Tr = Tc)
Tm = Pn ./ wn;
t_start = Jtot * wn ./ (Tm - Tc); % time needed to reach the steady speed 

% Finally the hourly energy is:
W_hour_2 = 0.5 * Pn * t_start + Pc .* 3600;

figure()
bar(a * 100, W_hour_2, 0.5)
grid on;
ylabel('Energy per hour [Wh]', 'interpreter', 'latex');
xlabel('Air flow percentage [\%]', 'interpreter', 'latex')
title('Speed control', 'interpreter', 'latex')


%% Comparison

figure()
bar(a * 100, W_hour_2, 0.1)
grid on;
hold on;
bar(a * 100 + 2, W_hour_1, 0.1)
legend('Speed control', 'Hysteresis control', 'location', 'nw')
ylabel('Energy per hour [Wh]', 'interpreter', 'latex');
xlabel('Air flow percentage [\%]', 'interpreter', 'latex')
title('Energy comparison', 'interpreter', 'latex')

% Considering the kWH cost I can calculate the cost for operating 1 year 
% (48 w/d * 5 d/w * 8 h/d)

C_tot_1 = W_hour_1 * 1e-3  * cost * 48 * 5 * 8 / 3600;
C_tot_2 = W_hour_2 * 1e-3 * cost * 48 * 5 * 8 / 3600;

figure()
bar(a * 100, C_tot_2, 0.1)
grid on;
hold on;
bar(a * 100 + 2, C_tot_1, 0.1)
legend('Speed control', 'Hysteresis control', 'location', 'nw')
ylabel('Cost for 1 year of operation', 'interpreter', 'latex');
xlabel('Air flow percentage [\%]', 'interpreter', 'latex')
title('Cost comparison', 'interpreter', 'latex')

cost_saved = abs(C_tot_1 - C_tot_2);
fprintf('\n\n-------------------------------------------------\n\t  The money saved are (case Pn = %d):\n\t  a = %3.2f -> %f €\n\t  a = %3.2f -> %f €\n\t  a = %3.2f -> %f €\n-------------------------------------------------\n\n', Pn, a(1), cost_saved(1), ...
     a(2), cost_saved(2), a(3), cost_saved(3));

