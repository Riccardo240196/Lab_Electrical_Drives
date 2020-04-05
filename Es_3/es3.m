clear all; close all; clc; 

%%  Data 

V = 300;                % [V]    Nominal DC supply voltage
Pn = 2800;              % [W]    Rated Power
fs = 8000;              % [Hz]   Switching frequency
V_GE = 17;              % [V]    IGBT gate voltage 
wn = 4100;              % [rpm]  Nominal speed
Pn_m = 2500;            % [W]    Rated power
Vn = 180;               % [V]    Rated voltage
Kt = 0.4;               % [Nm/A] Torque constant
R = 0.35;               % [Ohm]  Motor resistance
L = 1.25;               % [mH]   Motor inductance
Rd = 0.4;               % [°C/W] Heatsink thermal resistance
tau_th = 15*60;         % [s]    Heatsink thermal time constant
theta_amb = 45;         % [°C]   Ambient temperature

%% Duty cycle

time = [0 0.3999999999 0.4 2.999999999 3 5];
I = [35 35 25 25 0 0];
figure
plot(time,I,'linewidth',2);
grid on;
ylabel('Current [A]', 'interpreter', 'latex');
xlabel('Time [s]', 'interpreter', 'latex')
title('Current profile', 'interpreter', 'latex')

Omega = [0 4000 4000 4000 0 0];
figure
plot([0 0.3999999 0.4 2.999999999 3 5],Omega,'linewidth',2);
grid on;
ylabel('$\Omega$ [rpm]', 'interpreter', 'latex');
xlabel('Time [s]', 'interpreter', 'latex')
title('Speed profile', 'interpreter', 'latex')

%% Electrical analysis
% First of all, calculate the supply voltage V1 needed for the application 
% and then the duty cycle, that the control has to provide to the chopper 
% in order to follow the working operations.

V1 = R*I+Kt*pi/30*Omega;

figure
plot(time,V1,'linewidth',2);
grid on;
ylim([0 200])
ylabel('V1 [V]', 'interpreter', 'latex');
xlabel('Time [s]', 'interpreter', 'latex')
title('Qualitative profile of voltage V1', 'interpreter', 'latex')

delta = V1/V;

figure
plot(time,delta,'linewidth',2);
grid on;
ylim([0 1])
ylabel('$\delta$', 'interpreter', 'latex');
xlabel('Time [s]', 'interpreter', 'latex')
title('Qualitative profile of duty cycle', 'interpreter', 'latex')

% from the datasheet of the IGBT 600V class we can see that the maximum
% current is 100 A, so much above our 35 A.

%% Power losses
% In order to build a proper thermal analysis is necessary to evaluate both
% conduction and switching losses for the IGBT and the Diode. Then,
% considering the worst case condition, we check that the temperature of 
% each device is lower than the related limit.

I_int = @(t) interp1(time, I.^2, t);
d_int = @(t) interp1(time, delta, t);
Irms = sqrt(1/1.2*simpcomp(0, 5, 800, I_int));
d_avg = 1/5*simpcomp(0, 5, 800, d_int);

% From datasheet
V_CE = 1.8;        % [V]  Voltage drop in the IGBT     (conduction losses)
V_AK = 1.57;       % [V]  Voltage drop in the Diode    (conduction losses)
Eon = 1.7e-3;      % [Ws] Energy lost in on phase IGBT  (switching losses)
Eoff = 1.5e-3;     % [Ws] Energy lost in off phase IGBT (switching losses)
dI = 1250e6;       % [A/s] 

% Conduction losses
Pcond_IGBT = Irms*V_CE*d_avg;       % IGBT
Pcond_diode = Irms*V_AK*(1-d_avg);  % Diode

% Switching losses
Pswitch_IGBT = (Eon+Eoff)*fs;       % IGBT
Pswitch_diode = 0.5*V*Irms^2/dI;    % Diode

% Total losses
Ptot_IGBT = Pcond_IGBT + Pswitch_IGBT;      % IGBT
Ptot_diode = Pcond_diode + Pswitch_diode;   % diode

% Heatsink temperature
theta_HS = theta_amb + (Ptot_IGBT + Ptot_diode)*Rd;

%% Worst operating condition 
% Considering the switching frequency we can obtain the period and assuming
% that the 2% of this period is necessary to turn ON/OFF IGBT and Diode
% find the maximum delta. This assumption is fine since 0.02*T_sw = 2.5
% microseconds that is higher than the typical turn on/off time of the
% considered devices.

Irms_max = 35;
d_max = 0.98;
d_min = 0.02;
Rth_jc = 0.35;      % [K/W] from datasheet, IGBT
Rth_ch = 0.05;      % [K/W] from datasheet, Module 
Rth_jcD = 0.72;     % [K/W] from datasheet, Diode

theta_IGBT = theta_HS + Ptot_IGBT*(Rth_jc+Rth_ch);
theta_diode = theta_HS + Ptot_diode*(Rth_jcD+Rth_ch);

fprintf('\n\n-------------------------------------------------\n\t  Results obtained:\n\t  HS temperature = %3.1f °C\n\t  IGBT temperature = %3.1f °C\n\t  Diode temperature = %3.1f °C\n-------------------------------------------------\n\n',...
    theta_HS, theta_IGBT, theta_diode);

