clear all; close all; clc; 

%%  Data 
% The aim of this lab is to size a car battery meeting mass and autonomy 
% requirements. Two possibilities are analyzed: Lead-acid battery and
% Li-ion one.
% The initial data that we consider in this lab are the following:

v_max = 90/3.6;         % [m/s]     Maximum car speed
Vbatt = 80;             % [V]       Battery voltage
eta_batt = 0.8;         % [-]       Battery efficiency
DOD = 0.8;              % [-]       Depth of discharge (for Li-ion battery)
p = 0.1;                % [%]       Slope percentage
m = 300;                % [kg]      No load mass
mp = 75;                % [kg]      Mass of 1 person
ml = 40;                % [kg]      Luggage mass
mtot_max = 700;         % [kg]      Maximum total mass
s_max = 100e3;          % [m]       Autonomy to be achieved 
g = 9.81;               % [m/s^2]   Gravity acceleration
Cd = 0.3;               % [-]       Drag coefficient
f = 0.015;              % [-]       Rolling resistance coefficient
S=2.4;                  % [m^2]     Frontal section (for Drag)
rho = 1.2258;           % [kg/m^3]  Air density
Dw = 0.51;              % [m]       Wheel diameter
eta_tran = 0.95;        % [-]       Transmission efficiency
eta_mot = 0.92;         % [-]       Motor efficiency
tf = 240;               % [s]       Time lenght of 1 cycle
Q_kwh = 0.15;           % [€/kWh]   Electric energy cost

% Lead-acid battery specifications:
Esp_Pb = 30;            % [Wh/kg]   Specific energy (lead-acid battery)
Psp_Pb = 80;            % [Wh/kg]   Specific power  (lead-acid battery)
% Li-ion battery specifications:
Esp_Li = 120;           % [Wh/kg]   Specific energy (Li-ion battery)
Psp_Li = 240;           % [Wh/kg]   Specific power  (Li-ion battery)

v = @(t) interp1([0 7 47 54 84 98 198 212 240],[0 25 25 0 0 50 50 0 0]/3.6,t);
time = linspace(0,tf,10000);    
a = diff(v(time))./diff(time);
s = simpcomp(0, tf, 400, v);

figure
subplot(2,1,1);
plot(time,v(time),'linewidth',2); grid on
title('Speed profile','interpreter', 'latex');
xlabel('time [s]', 'interpreter', 'latex');
ylabel('speed [m/s]', 'interpreter', 'latex');
subplot(2,1,2);
plot(time,[a 0],'linewidth',2); grid on
xlabel('time [s]', 'interpreter', 'latex');
ylabel('acceleration [m/$s^{2}$]', 'interpreter', 'latex');

%% Urban route with no slope
% First, we consider the urban cycle with no slope, in order to size both
% the two batteries. The process is iterative since at the beginning the
% battery mass is unknown. We start considering the maximum allowable mass.

theta = 0;
mtot = mtot_max;
n_cycle = s_max/s;
%% Forces calculation
% The forces to be considered and evaluated are the inertial one, the
% gravitational, the one related to the rolling resistance and the
% aerodinamic force.

Fin = mtot.*[a 0];
Fg = mtot*g*sin(theta);
Frr = mtot*g*f*cos(theta);
Faero = 0.5*rho*S*Cd.*(v(time).^2);
Ftot = Fin + Fg + Frr + Faero;

%% Power and Energy required
% In order to properly size the batteries is necessary to evaluate the 
% amount of power and energy required to complete one cycle. Then,
% considering mass and autonomy constraints we can eventually find the
% battery mass and redo all the calculation with the real total mass.

P = @(t) interp1(time, Ftot.*v(time) , t);
Ecycle = simpcomp(0, tf, 400, P)/3600;
Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
Ebatt_c_Pb = Ebatt_c;
Ebatt_c_Li = Ebatt_c/DOD;

Ebatt_totPb = Ebatt_c_Pb*n_cycle;
Ebatt_totLi = Ebatt_c_Li*n_cycle;

% Li-ion Battery: iterative process
mbatt_E = Ebatt_totLi/Esp_Li;
mbatt_P = max(P(time))/Psp_Li;
mbatt_in = max(mbatt_E,mbatt_P);
mbatt_out = mbatt_in + 1;

while abs(mbatt_in-mbatt_out) > 1e-3 
    mbatt_in = mbatt_out;
    mtot = m + 2*mp + ml + mbatt_in;
    Ftot = mtot*([a 0]+g*sin(theta)+g*f*cos(theta)) + 0.5*rho*S*Cd.*(v(time).^2);
    P = @(t) interp1(time, Ftot.*v(time) , t);
    Ecycle = simpcomp(0, tf, 400, P)/3600;
    Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
    Ebatt_c_Li = Ebatt_c/DOD;
    Ebatt_totLi = Ebatt_c_Li*n_cycle;
    mbatt_E = Ebatt_totLi/Esp_Li;
    mbatt_P = max(P(time))/Psp_Li;
    mbatt_out = max(mbatt_E,mbatt_P);
end

if mtot <= mtot_max
    mbatt_Li = mbatt_out;
    mtot_Li = mtot;
else
    mtot_Li = mtot_max;
    Ftot = mtot_Li*([a 0]+g*sin(theta)+g*f*cos(theta)) + 0.5*rho*S*Cd.*(v(time).^2);
    P = @(t) interp1(time, Ftot.*v(time) , t);
    Ecycle = simpcomp(0, tf, 400, P)/3600;
    Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
    Ebatt_c_Li = Ebatt_c;
    mbatt_Li = mtot_max - (m + 2*mp + ml);
    Ebatt_totLi = Esp_Li*mbatt_Li;
    n_cycleLi = Ebatt_totLi/Ebatt_c_Li;
    smax_Li = s*n_cycleLi;
end

figure()
subplot(2, 1, 1)
plot(time, Ftot, 'linewidth', 2);
grid on; 
title('Urban drive cycle (slope p = 0\%) - Li-ion battery', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Force [N]', 'Interpreter', 'latex');
subplot(2, 1, 2)
plot(time, P(time), 'linewidth', 2);
grid on; 
title('Urban drive cycle (slope p = 0\%) - Li-ion battery', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Power [W]', 'Interpreter', 'latex');

C_recharge_Li = (Ebatt_totLi * 1e-3) * Q_kwh;

fprintf('\n\n------------------------------------------\n\n\t')
fprintf('Li-ion Battery:\n\n\t')
fprintf('E_{spec} = %3.0f  [Wh/kg]\n\t',Esp_Li)
fprintf('E_{cycle} = %3.2f [Wh]\n\t',Ebatt_c_Li)
fprintf('E_{batt} = %4.2f  [Wh]\n\t',Ebatt_totLi)
fprintf('P_{spec} = %3.0f  [Wh/kg]\n\t',Psp_Li)
fprintf('P_{max} = %5.1f   [W]\n\t',max(P(time)))
fprintf('m_{batt}(s_{max}) = %2.2f  [kg]\n\t',mbatt_out)
fprintf('s(m_{max}) = %2.2f  [km]\n\t',s_max*1e-3)
fprintf('C_{recharge} = %2.2f  [€]\n\t',C_recharge_Li)
fprintf('\n------------------------------------------\n\n')

% Lead-acid Battery: iterative process
mbatt_E = Ebatt_totPb/Esp_Pb;
mbatt_P = max(P(time))/Psp_Pb;
mbatt_in = max(mbatt_E,mbatt_P);
mbatt_out = mbatt_in;
mtot = m + 2*mp + ml + mbatt_in;

% while abs(mbatt_in-mbatt_out) > 1e-3 
%     mbatt_in = mbatt_out;
%     mtot = m + 2*mp + ml + mbatt_in;
%     Ftot = mtot*([a 0]+g*sin(theta)+g*f*cos(theta)) + 0.5*rho*S*Cd.*(v(time).^2);
%     P = @(t) interp1(time, Ftot.*v(time) , t);
%     Ecycle = simpcomp(0, tf, 400, P)/3600;
%     Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
%     Ebatt_c_Pb = Ebatt_c;
%     Ebatt_totPb = Ebatt_c_Pb*n_cycle;
%     mbatt_E = Ebatt_totPb/Esp_Pb;
%     mbatt_P = max(P(time))/Psp_Pb;
%     mbatt_out = max(mbatt_E,mbatt_P);
% end

if mtot <= mtot_max
    mbatt_Pb = mbatt_out;
    mtot_Pb = mtot;
else
    mtot_Pb = mtot_max;
    Ftot = mtot_Pb*([a 0]+g*sin(theta)+g*f*cos(theta)) + 0.5*rho*S*Cd.*(v(time).^2);
    P = @(t) interp1(time, Ftot.*v(time) , t);
    Ecycle = simpcomp(0, tf, 400, P)/3600;
    Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
    Ebatt_c_Pb = Ebatt_c;
    mbatt_Pb = mtot_max - (m + 2*mp + ml);
    Ebatt_totPb = Esp_Pb*mbatt_Pb;
    n_cyclePb = Ebatt_totPb/Ebatt_c_Pb;
    smax_Pb = s*n_cyclePb;
end


figure()
subplot(2, 1, 1)
plot(time, Ftot, 'linewidth', 2);
grid on; 
title('Urban drive cycle (slope p = 0\%) - Lead-acid battery', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Force [N]', 'Interpreter', 'latex');
subplot(2, 1, 2)
plot(time, P(time), 'linewidth', 2);
grid on; 
title('Urban drive cycle (slope p = 0\%) - Lead-acid battery', 'Interpreter', 'latex');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Power [W]', 'Interpreter', 'latex');

C_recharge_Pb = (Ebatt_totPb * 1e-3) * Q_kwh;

fprintf('\n\n------------------------------------------\n\n\t')
fprintf('Lead-acid Battery:\n\n\t')
fprintf('E_{spec} = %3.0f  [Wh/kg]\n\t',Esp_Pb)
fprintf('E_{cycle} = %3.2f [Wh]\n\t',Ebatt_c_Pb)
fprintf('E_{batt} = %4.2f  [Wh]\n\t',Ebatt_totPb)
fprintf('P_{spec} = %3.0f  [Wh/kg]\n\t',Psp_Pb)
fprintf('P_{max} = %5.1f   [W]\n\t',max(P(time)))
fprintf('m_{batt}(s_{max}) = %2.2f  [kg]\n\t',mbatt_out)
fprintf('s(m_{max}) = %2.2f  [km]\n\t',smax_Pb*1e-3)
fprintf('C_{recharge} = %2.2f  [€]\n\t',C_recharge_Pb)
fprintf('\n------------------------------------------\n\n')

%% Urban route with positive slope
% Now, we consider the urban cycle with a 10% positive slope. Considering
% this new configuration, is possible to evaluate the new forces, power
% and energy required. Eventually, the new autonomy is found.

theta = atan(p);
% Li-ion Battery:
Ftot = mtot_Li*([a 0]+g*sin(theta)+g*f*cos(theta)) + 0.5*rho*S*Cd.*(v(time).^2);
P = @(t) interp1(time, Ftot.*v(time) , t);
Ecycle = simpcomp(0, tf, 400, P)/3600;
Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
Ebatt_c_Li = Ebatt_c/DOD;
n_cycleLi_pend = Ebatt_totLi/Ebatt_c_Li;
s_maxLi_pend = s*n_cycleLi_pend;

% Lead-acid Battery:
Ftot = mtot_Pb*([a 0]+g*sin(theta)+g*f*cos(theta)) + 0.5*rho*S*Cd.*(v(time).^2);
P = @(t) interp1(time, Ftot.*v(time) , t);
Ecycle = simpcomp(0, tf, 400, P)/3600;
Ebatt_c = Ecycle/eta_batt/eta_mot/eta_tran;
Ebatt_c_Pb = Ebatt_c;
n_cyclePb_pend = Ebatt_totPb/Ebatt_c_Pb;
s_maxPb_pend = s*n_cyclePb_pend;

fprintf('\n\n------------------------------------------\n\n\t')
fprintf('Lead-acid Battery:\n\t')
fprintf('s_{pend} = %2.2f  [km]\n\t',s_maxPb_pend*1e-3)
fprintf('Li-ion Battery:\n\t')
fprintf('s_{pend} = %2.2f  [km]\n\t',s_maxLi_pend*1e-3)
fprintf('\n------------------------------------------\n\n')

%% Extra-Urban cycle
% At last, we want to determine the autonomy of the battery considering an
% extra-urban cycle performed with constant speed equal to 90 km/h and with 
% no slope. 

% Li-ion Battery:
Ftot = mtot_Li*g*f + 0.5*rho*S*Cd.*(v_max.^2);
P = Ftot.*v_max;
T = Ebatt_totLi*3600/P;
s_maxLi_ex = T*v_max;

% Lead-acid Battery:
Ftot = mtot_Pb*g*f + 0.5*rho*S*Cd.*(v_max.^2);
P = Ftot.*v_max;
T = Ebatt_totPb*3600/P;
s_maxPb_ex = T*v_max;

fprintf('\n\n------------------------------------------\n\n\t')
fprintf('Lead-acid Battery:\n\t')
fprintf('s_{pend} = %2.2f  [km]\n\t',s_maxPb_ex*1e-3)
fprintf('Li-ion Battery:\n\t')
fprintf('s_{extra} = %2.2f  [km]\n\t',s_maxLi_ex*1e-3)
fprintf('\n------------------------------------------\n\n')

% Since the lead-acid battery cannot satisfy both the constraints on 
% autonomy and mass, the lithium-ion kind of batteries is chosen.