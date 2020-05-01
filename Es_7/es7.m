close all
clear all
clc

%% Data
m = 300;            % [kg]          Mass of the object 
Fres = 2000;        % [N]           Resistance force
Jtrans = 250e-6;    % [kg*m^2]      Transmission M.O.I
stroke = 0.15;      % [m]           Stroke 
omega_max = 700;    % [rad/s]       Max angular speed
tA = 0.4;           % [s]           Time to transport the mass

% The chosen motor is the Ultract II - 503402. Data are reported below.
Jm = 1e-4;          % [kg*m^2]      Motor M.O.I
Tn = 3.47;          % [Nm]          Nominal torque, DT=65°C
Tpk = 11.56;        % [Nm]          Peak Torque
omega_n = 419;      % [rad/s]       Nominal speed
tau = 2001;         % [s]           Thermal time constant
Rth = 0.637;        % [°C/W]        Thermal impedance
Cth = 3140;         % [J/°C]        Thermal capacity
L0 = 40;            % [W]           No load losses at base speed
L = 6.94e-3/2;      % [H]           Motor winding inductance
Rw = 2.2;           % [Ohm]         Winding resistance @ 20°C
Kt = 0.72;          % [Nm/Arms]     Torque constant @ 20°C
n = 4;              % [-]           Number of pole pairs

%% Pseudosinusoidal Speed profile

a_max = 6.45; 
time = 0:0.001:1.2;
t1 = 0:0.001:0.14;
a1 = sin(pi*t1/0.14);
t2 = 0.26:0.001:0.4;
a2 = -sin(pi*(t2-0.26)/0.14);
t3 = 0.6:0.001:0.74;
a3 = -sin(pi*(t3-0.6)/0.14);
t4 = 0.86:0.001:1;
a4 = sin(pi*(t4-0.86)/0.14);
a = a_max.*[a1 zeros(1,120) a2 zeros(1,200-1) a3  zeros(1,120-1) a4  zeros(1,200-1)];
a_int = @(t) interp1(time(1:end),a,t);

[t_ode, v_ode] = ode45(@(t, v) a_int(t), 0:0.014:time(end), 0);
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

v_avg = 1/tA*simpcomp(0, tA, 200, v);
v_max = max(v(time));  

%% Torque Required
% First, the required torque is calculated assuming the motor inertia as
% 20% of the load one. Then, the motor is chosen and this assumption will
% be verified.

k = 0.8*omega_max/v_max;        % Gear ratio
%k = 1.1*419/v_max;             % Gear ratio
Jmass = m/(k^2);                % Mass M.O.I
Jmot = 0.176*(Jtrans+Jmass);    % Assumption
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
Tavg = mean(Tm);

%% Converter sizing
% Since the electric motor is a three-phase low voltage synchronous motor, 
% the most suitable converter is a three-phase inverter made by IGBT.
% To define the size of the converter, the procedure is similar to the one 
% followed for the sizing of the electric motor; we are going to rely on 
% the current absorbed by the motor during its working cycle.

v = v(time);
v(1192:end) = 0;
w = k.*v;
w_int = @(t) interp1(time, abs(w), t);

%% Premliminary Checks
% In order to size the converter, some preliminary checks are made.
% First of all, the converter output voltage has to be higher or equal to 
% the motor nominal one and the supply voltage conforms to the available 
% line. Then, we have to verify that the nominal output power of the 
% converter is higher than the motor nominal one.

% Maximum frequency
w_max = max(w);
fmax = w_max*n/2/pi;
% Maximum voltage
Vin = 230;                      % [V]  Input AC voltage
theta_max = 130;                % [°C] Maximum allowable temperature
theta_0 = 20;                   % [°C] Reference temperature 
Kt_hot = Kt-(0.09*(theta_max-theta_0)/100);
Rph_max = (234.5+theta_max)/(234.5+theta_0)*Rw/2;
Imax_hot = max(Tm)/Kt_hot;
Vm = sqrt(3)*sqrt((Rph_max*Imax_hot + w_max*Kt_hot/3)^2 + (n*w_max*L*Imax_hot)^2);
Vmax = 0.98*Vin;

w_Tmax = w(find(Tm==max(Tm)));
Iw_max = Tm(141)/Kt_hot;

if Vm > Vmax
    Vm_T = sqrt(3)*sqrt((Rph_max*Imax_hot + w_Tmax*Kt_hot/3)^2 + (n*w_Tmax*L*Imax_hot)^2);
    Vm_w = sqrt(3)*sqrt((Rph_max*Iw_max + w_max*Kt_hot/3)^2 + (n*w_max*L*Iw_max)^2);
end

% Current 
Irms = Trms/Kt_hot;
Iavg = Tavg/Kt_hot;
Imax = max(Tm)/Kt_hot;

fprintf('\n\n---------------------------------------------------------------------\n\n\t')
fprintf('- Frequency check result:\n\t')
fprintf('Converter maximum frequency: fmax = %3.1f Hz\n\t',450)
fprintf('Maximum operating frequency:    f = %3.1f Hz\n\n\t',fmax)
fprintf('Since the max operating frequency is lower than the max one this check is satisfied\n\t')
fprintf('\n---------------------------------------------------------------------\n\n\t')
fprintf('- Voltage check results:\n\t')
fprintf('Converter maximum voltage: Vmax = %3.1f V\n\t',Vmax)
fprintf('Maximum phase voltage:       Vm = %3.1f V\n\t',Vm)
fprintf('Phase voltage @ Tmax:      Vm_T = %3.1f V\n\t',Vm_T)
fprintf('Phase voltage @ Wmax:      Vm_w = %3.1f V\n\n\t',Vm_w)
fprintf('Since both the two phase voltages are lower than the max one this check is satisfied\n\t')
fprintf('\n---------------------------------------------------------------------\n\n\t')
fprintf('- Current check results:\n\t')
fprintf('RMS operating current:     Irms = %3.1f  Arms\n\t',Irms)
fprintf('Average operating current: Iavg = %3.1f  A\n\t',Iavg)
fprintf('Maximum operating current: Imax = %3.1f A\n\n\t',Imax)
fprintf('Considering these results the converter chosen is the XVy 10612.\n\t')
fprintf('\n---------------------------------------------------------------------\n\n\t')

%% Thermal analysis
% In this section, the converter is thermally analyzed in order to evaluate
% its time constant that has to be much higher than the overload time
% related to the current cycle.

I = abs(Tm)/Kt_hot;
In = 6;                 % [Arms] From datasheet
Imax_c = max(I);

figure
plot(time,I,'linewidth',2); grid on; hold on
plot(time,In*ones(size(time,2),1),'linewidth',2); grid on
xlabel('time [s]', 'interpreter', 'latex');
ylabel('Current [A]', 'interpreter', 'latex');
title('Current profile','interpreter', 'latex');

tt = find(I>In);
dt = time(tt(end))-time(tt(1));
tau1 = -dt/log((Imax_c^2-In^2)/Imax_c^2);

% Since tau is not at least 10 times higher than tov, we have to consider
% also the overheating.

tov = 1;                % [s]    From datasheet
Iov = 12;               % [Arms] From datasheet
tau = -tov/log((Iov^2-In^2)/Iov^2);

fprintf('\n\n---------------------------------------------------------------------\n\n\t')
fprintf('- Thermal result:\n\t')
fprintf('Overload time considering the duty cycle:  dt = %3.2f s\n\t',dt)
fprintf('Maximum overload time (from datasheet):   tov = %3.1f s\n\t',tov)
fprintf('Converter time constant (no overheating): tau = %3.1f s\n\t',tau1)
fprintf('Converter time constant (overheating):    tau = %3.1f s\n\t',tau)
fprintf('Since the last tau is much higher than dt, the converter is thermally verified\n\t')
fprintf('\n---------------------------------------------------------------------\n\n\t')

%% Braking resistance
% When the motor is braking, the power is injected in the converter and has
% to be dissipated. Thus, is necessary to size the braking resistance.

P = Tm.*w;

figure
plot(time,P,'linewidth',2); grid on; hold on
xlabel('time [s]', 'interpreter', 'latex');
ylabel('Power [W]', 'interpreter', 'latex');
title('Power profile','interpreter', 'latex');

Ebr = 11e3;             % [J]   From datasheet
Pnbr = 100;             % [W]   From datasheet
Tn_brk = Ebr/Pnbr;
tt = find(P(1:400)<0);
dt1 = time(tt(end))-time(tt(1));
Ebrk1 = simpcomp(time(tt(1)), time(tt(end)), 400, @(t) P);

tt = find(P(800:end)<0);
dt2 = time(tt(end))-time(tt(1));
Ebrk2 = simpcomp(time(tt(1)), time(tt(end)), 400, @(t) P);

Ebrk = Ebrk1+Ebrk2;
Pavg = Ebrk/1.2;

fprintf('\n\n---------------------------------------------------------------------\n\n\t')
fprintf('- Results:\n\t')
fprintf('First braking time interval:                   dt,1 = %3.2f s\n\t',dt1)
fprintf('Second braking time interval:                  dt,2 = %3.2f s\n\t',dt2)
fprintf('First braking energy of the duty cycle:     E_brk,1 = %3.2f J\n\t',Ebrk1)
fprintf('Second braking energy of the duty cycle:    E_brk,2 = %3.2f J\n\t',Ebrk2)
fprintf('Total braking energy of the duty cycle:       E_brk = %3.2f J\n\t',Ebrk)
fprintf('Average power of the working cycle (E_br/T):  P_brk = %3.2f W\n\t',Pavg)
fprintf('Considering that the braking times that have to be lower than the max dissipation time, ')
fprintf('the braking energies of the working cycle lower than the maximum allowable one ')
fprintf('and the average power lower than the nominal power of the resistance, ')
fprintf('the chosen resistance is the CBR-100R. These are its data:\n\t')
fprintf('Nominal braking power (from datasheet):       P_Nbr = %3.1f W\n\t',Pnbr)
fprintf('Maximum energy to be dissipated (datasheet):   E_br = %3.1f J\n\t',Ebr)
fprintf('Maximum dissipation time (E_br/P_Nbr):         Tbrk = %3.2f s\n\t',Tn_brk)
fprintf('\n---------------------------------------------------------------------\n\n\t')

%% Steady state analysis
% First, is necessary to calculate iron and copper losses that are then
% summed to obtain the total power losses. Then, with an iterative process 
% the motor temperature is calculated.

% theta_amb = 65;                                 % [°C] Ambient temperature 
% Kfe = L0/omega_n;                               % Iron losses coefficient 
% Pfe = Kfe*1/1.2*simpcomp(0, 1.2, 800, w_int);   % Average iron power losses
% Irms = Trms/Kt;                                 % RMS current
% R_ph_in = Rw/2;                                 % Winding resistance
% toll = 1e-4;                                    % Tolerance
% thetaM_in = 50;                                 % First guess
% thetaM_out = thetaM_in+10*toll;                 % First guess
% 
% while abs(thetaM_in - thetaM_out) > toll
%     R_ph = R_ph_in;
%     Pcu = 3*R_ph*Irms^2;
%     Ptot = Pfe+Pcu;
%     thetaM_in = Ptot*Rth+theta_amb;
%     R_ph_out = (234.5+thetaM_in)/(234.5+theta_0)*Rw/2;
%     Pcu = 3*R_ph_out*Irms^2;
%     Ptot = Pfe+Pcu;
%     thetaM_out = Ptot*Rth+theta_amb;
%     R_ph_in = R_ph_out;
% end
% 
% if thetaM_out > theta_max
%     disp('The motor is not thermally verified at s.s. service, try increasing Kt. \n')
% else
%     fprintf('The motor temperature is %2.1f°C and so is thermally verified at s.s. service. \n', thetaM_out)
% end

