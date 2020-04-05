close all
clear all
clc

%% Data
m = 300;            % [kg]      Mass of the object 
Fres = 2000;        % [N]       Resistance force
Jtrans = 250e-6;    % [kg*m^2]  Transmission M.O.I
stroke = 0.15;      % [m]       Stroke 
omega_max = 700;    % [rad/s]   Max angular speed
tA = 0.4;           % [s]       Time to transport the mass

%% Pseudosinusoidal Speed profile

a_max = 5.78; % con a_int commentata 5.25, 6.825
% a_int = @(t) interp1([0 0.04 0.12 0.16 0.24 0.28 0.36 0.4 0.6 0.64 0.72 0.76 0.84 0.88 0.96 1 1.2],[0 a_max a_max 0 0 -a_max -a_max 0 0 -a_max -a_max 0 0 a_max a_max 0 0],t);
% a_int = @(t) interp1([0 0.04 0.08 0.12 0.28 0.32 0.36 0.4 0.6 0.64 0.68 0.72 0.88 0.92 0.96 1 1.2],[0 a_max a_max 0 0 -a_max -a_max 0 0 -a_max -a_max 0 0 a_max a_max 0 0],t);
a_int = @(t) interp1([0 0.04 0.1 0.14 0.26 0.3 0.36 0.4 0.6 0.64 0.7 0.74 0.86 0.9 0.96 1 1.2],[0 a_max a_max 0 0 -a_max -a_max 0 0 -a_max -a_max 0 0 a_max a_max 0 0],t);
time = linspace(0,1.2,1000);
[t_ode, v_ode] = ode45(@(t, v) a_int(t), 0:0.01:time(end), 0);
v = @(t) interp1(t_ode,v_ode,t);
stroke2 = simpcomp(0, 0.4, 200, v);
a = a_int(time(1:end-1));
tACC = 0.06;
figure
subplot(2,1,1);
plot(time,v(time),'linewidth',2); grid on
title('Pseudosinusoidal speed profile','interpreter', 'latex');
xlabel('time [s]', 'interpreter', 'latex');
ylabel('speed [m/s]', 'interpreter', 'latex');
subplot(2,1,2);
plot(time,a_int(time),'linewidth',2); grid on
xlabel('time [s]', 'interpreter', 'latex');
ylabel('acceleration [m/$s^{2}$]', 'interpreter', 'latex');

% v_avg = stroke/tA;
v_avg = 1/tA*simpcomp(0, tA, 200, v);
v_max = max(v(time));  

%% Torque required
% First, the required torque is calculated assuming the motor inertia as
% 20% of the load one. Then, the motor is chosen and this assumption will
% be verified.

k = 0.9*omega_max/v_max;        % Gear ratio
Jmass = m/(k^2);                % Mass M.O.I
Jmot = 0.198*(Jtrans+Jmass);    % Assumption
Jm = 1e-4;                      % Motor M.O.I
Jmot = Jm;
Jtot = (Jmot+Jtrans+Jmass);     % Total M.O.I
Tres = Fres/k;                  % Resistance torque
omega_avg = k*v_avg;            % Average angular speed
omega_p = k*[a 0];              % Angular acceleration

for i = 1:length(time)
    if time(i) < 0.39992
        Tm(i) = Jtot*omega_p(i) + Tres;
    elseif time(i) > 0.4 && time(i) < 0.6
        Tm(i) = Jtot*omega_p(i);
    else 
        Jtot = (Jmot+Jtrans);  
        Tm(i) = Jtot*omega_p(i);
    end
end

figure
plot(time,Tm,'linewidth',2); grid on
xlabel('time [s]', 'interpreter', 'latex');
ylabel('Torque [Nm]', 'interpreter', 'latex');
title('Torque profile','interpreter', 'latex');

Tm_int = @(t) interp1(time, Tm.^2, t);
Trms = sqrt(1/1.2*simpcomp(0, 1.2, 800, Tm_int));

%% Motor Choosing
% The chosen motor is the Ultract II - 503402. Data are reported below.

Jm = 1e-4;          % [kg*m^2]      Motor M.O.I
Tn = 3.47;          % [Nm]          Nominal torque, DT=65°C
Tpk = 11.56;        % [Nm]          Peak Torque
omega_n = 419;      % [rad/s]       Nominal speed
tau = 2001;         % [s]           Thermal time constant
Rth = 0.637;        % [°C/W]        Thermal impedance
Cth = 3140;         % [J/°C]        Thermal capacity
L0 = 40;            % [W]           No load losses at base speed
Rw = 2.2;           % [Ohm]         Winding resistance @ 20°C
Kt = 0.72;          % [Nm/Arms]     Torque constant @ 20°C

fprintf('\n\n---------------------------------------------------------\n\t  Constraints:\n\t  Trms = %1.2f N < Tn = %1.2f N\n\t  Tmax = %1.2f N < Tpk = %1.2f N\n\t  omega_avg = %3.1f rad/s < omega_n = %3.1f rad/s\n---------------------------------------------------------\n\n',...
    Trms, Tn, max(Tm), Tpk, omega_avg, omega_n);

%% Thermal analysis
% The thermal analysis is carried out three times; first at steady state,
% then considering an intermitted duty and at last in continuos service.

theta_max = 130;                % [°C] Maximum allowable temperature
theta_amb = 65;                 % [°C] Ambient temperature 
theta_0 = 20;                   % [°C] Reference temperature 

%% Steady state analysis
% First, is necessary to calculate iron and copper losses that are then
% summed to obtain the total power losses. Then, with an iterative process 
% the motor temperature is calculated.

Kfe = L0/omega_n;               % Iron losses coefficient 
Pfe_avg = Kfe*omega_avg;        % Average iron power losses
Irms = Trms/Kt;                 % RMS current
R_ph_in = Rw/2;                 % Winding resistance
toll = 1e-4;                    % Tolerance
thetaM_in = 50;                 % First guess
thetaM_out = thetaM_in+10*toll; % First guess

while abs(thetaM_in - thetaM_out) > toll
    R_ph = R_ph_in;
    Pcu = 3*R_ph*Irms^2;
    Ptot = Pfe_avg+Pcu;
    thetaM_in = Ptot*Rth+theta_amb;
    R_ph_out = (234.5+thetaM_in)/(234.5+theta_0)*Rw/2;
    Pcu = 3*R_ph_out*Irms^2;
    Ptot = Pfe_avg+Pcu;
    thetaM_out = Ptot*Rth+theta_amb;
    R_ph_in = R_ph_out;
end

if thetaM_out > theta_max
    disp('The motor is not thermally verified at s.s. service, try increasing Kt. \n')
else
    fprintf('The motor temperature is %2.1f°C and so is thermally verified at s.s. service. \n', thetaM_out)
end

%% Intermitted duty 
% In this section, the overall cycle last for 600 s and as a consequence
% its duration is comparable with the thermal time constant of the motor.
% For this reason the transient has to be analyzed, verifying that the
% temperature does not go above 130°C.

ton = 180;
T = 600;
timeON = linspace(0,ton,10000);
timeOFF = linspace(ton,T,10000);
% theta_min = (theta_max-theta_amb)*exp(-(T-ton)/tau);
% A = (theta_max-theta_amb-theta_min)/(exp(-(ton)/tau)-1);
% Ptot_int = (theta_min-A)/Rth;
Aon = (theta_max-theta_amb)*(exp(-(T-ton)/tau)-1)/(1-exp(-(ton)/tau));
PtotON = ((theta_max-theta_amb)-Aon*exp(-(ton)/tau))/Rth;
thetaON = @(t) Aon*exp(-t/tau)+Rth*PtotON;
thetaOFF = @(t) (theta_max-theta_amb)*exp(-(t-ton)/tau);

figure
plot(timeON,theta_amb+thetaON(timeON),'-r','linewidth',2); hold on;
plot(timeOFF,theta_amb+thetaOFF(timeOFF),'-r','linewidth',2);
xlabel('time [s]', 'interpreter', 'latex');
ylabel('motor temperature [$^{\circ}$C]', 'interpreter', 'latex');
title('Motor temperature profile: intermitted duty','interpreter', 'latex');
ylim([110 135])

%% Maximum mass weigth
% Since the cycle is different, is possible to evaluate the new RMS torque
% and the consequent new total moment of inertia. Then, considering the
% gear ratio we can calculate the new maximum transportable mass.

thetaON_int = @(t) interp1(timeON, thetaON(timeON), t);
thetaOFF_int = @(t) interp1(timeOFF, thetaOFF(timeOFF), t);
theta_avg = 1/T*(simpcomp(0, ton, 800, thetaON_int)+simpcomp(ton, T, 800, thetaOFF_int));
R = (234.5+theta_avg)/(234.5+theta_0)*Rw/2; 
Pcu = PtotON-Pfe_avg;
Irms = sqrt(Pcu/(3*R));
Trms_new = Kt*Irms;
Trms_old = Trms;
Jtot_max = Jtot;
j = 1;
bool = 0;

% Here we calculate the new total moment of inertia.
while bool == 0 
    for i = 1:length(time)
        if time(i) < 0.3999
            Tm(i) = Jtot_max*omega_p(i) + Tres;
        elseif time(i) > 0.4 && time(i) < 0.6
            Tm(i) = 0;
        elseif time(i) > 0.6 && time(i) < 1
            Tm(i) = Jtot_max*omega_p(i);
        else
            Tm(i) = 0;
        end
    end
    Tm_int = @(t) interp1(time, Tm.^2, t);
    Trms_old = sqrt(1/1.2*simpcomp(0, 1.2, 800, Tm_int));
    if Trms_new > Trms_old
        Jtot_max = j*Jtot;
    end
    j = j+0.25;
    delta = (Trms_new - Trms_old);
    if delta > 0.1 && delta > -0.1
        bool = 0;
    else
        bool = 1;
    end
end

m_max = (Jtot_max-Jtrans-Jmot)*k^2;
Tmax = max(Tres + Jtot_max.*omega_p);
if Tmax < Tpk
    fprintf('Also considering the new maximum mass %3.1f kg, the max torque is %2.2f and is less than the peak torque. \n',m_max,  Tmax)
else
    fprintf('Considering the new maximum mass %3.1f, the max torque is higher than Tpk. Trying with another method: \n',m_max)
    Jtot_max = (Tpk-Tres)/max(omega_p);
    m_max = (Jtot_max-Jtrans-Jmot)*k^2;
    Tmax = max(Tres + Jtot_max.*omega_p);
    fprintf('The max mass is %3.1f and the max torque is %2.2f.\n',m_max,  Tmax)
end

%% Limited time duty
% Here, the machine has to work for 30 minutes continuously assuming that 
% at the end of the cycle the overtemperature will reached the maximum one.
% Then, as done before, we are looking for the maximum admittable torque,
% and consequently for the maximum transportable mass.

T = 30*60;
time_lim = linspace(0,T,10000);
Ptot_lim = (theta_max-theta_amb)/(1-exp(-T/tau))/Rth;
A = -Ptot_lim*Rth;
theta_lim = @(t) A*exp(-t/tau)+Rth*Ptot_lim;
figure
plot(time_lim,theta_amb+theta_lim(time_lim),'-r','linewidth',2); 
xlabel('time [s]', 'interpreter', 'latex');
ylabel('motor temperature [$^{\circ}$C]', 'interpreter', 'latex');
title('Motor temperature profile: limited time duty','interpreter', 'latex');

% Maximum mass weigth
theta_avg_lim = 1/T*(simpcomp(0, T, 800, theta_lim));
R = (234.5+theta_avg_lim)/(234.5+theta_0)*Rw/2; 
Pcu = Ptot_lim-Pfe_avg;
Irms = sqrt(Pcu/(3*R));
Trms_lim_new = Kt*Irms;
Trms_old = Trms;
Jtot_max_lim = Jtot;
j = 1;
bool = 0;

% Here we calculate the new total moment of inertia.
while bool == 0
    for i = 1:length(time)
        if time(i) < 0.3999
            Tm(i) = Jtot_max_lim*omega_p(i) + Tres;
        elseif time(i) > 0.4 && time(i) < 0.6
            Tm(i) = 0;
        elseif time(i) > 0.6 && time(i) < 1
            Tm(i) = Jtot_max_lim*omega_p(i);
        else
            Tm(i) = 0;
        end
    end
    Tm_int = @(t) interp1(time, Tm.^2, t);
    Trms_old = sqrt(1/1.2*simpcomp(0, 1.2, 800, Tm_int));
    if Trms_new > Trms_old
        Jtot_max_lim = j*Jtot;
    end
    j = j+0.25;
    delta = (Trms_lim_new - Trms_old);
    if delta > 0.1 && delta > -0.1
        bool = 0;
    else
        bool = 1;
    end
end

m_max_lim = (Jtot_max_lim-Jtrans-Jmot)*k^2;
Tmax_lim = max(Tres + Jtot_max_lim.*omega_p);
if Tmax_lim < Tpk
    fprintf('Also considering the new maximum mass %3.1f kg, the max torque is %2.2f and is less than the peak torque.',m_max_lim,  Tmax_lim)
else
    fprintf('Considering the new maximum mass %3.1f, the max torque is higher than Tpk. Trying with another method: \n',m_max_lim)
    Jtot_max = (Tpk-Tres)/max(omega_p);
    m_max_lim = (Jtot_max_lim-Jtrans-Jmot)*k^2;
    Tmax_lim = max(Tres + Jtot_max.*omega_p);
    fprintf('The max mass is %3.1f and the max torque is %2.2f .',m_max_lim,  Tmax_lim)
end
