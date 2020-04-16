clear all; close all; clc; 

%%  Data 
% The aim of this lab is to defining the operating range for a PMSM 
% (AC brushless) for small traction application.
% The initial data that we consider in this lab are the following:

Vbatt = 48;             % [V]       Battery voltage
Vn = 28;                % [Vrms]    Nominal motor voltage
wn = 141;               % [rad/s]   Nominal motor speed
Imax = 75;              % [Arms]    Maximum inverter current
fmax = 250;             % [Hz]      Maximum inverter switching frequency
eta_inv = 0.95;         % [-]       Inverter efficiency
n = 4;                  % [-]       Pole pairs number
Ke = 0.042;             % [V/s]     Back emf cpnstant
Kt = n*Ke;              % [Nm/A]    Torque constant
Rs = 0.05;              % [Ohm]     Motor resistance
Ls = 1.9e-4;            % [H]       Motor inductance
Pfe_n = 6.8;            % [W]       Nominal iron power losses 
z = 6;                  % [-]       Gear ratio
r = 0.25;               % [m]       Wheel radius
eta_tran = 0.97;        % [-]       Transmission efficiency

%% Relevant parameters evaluation
% The calculation has to be performed as a function of the frequency f, 
% that starts from a minimum of 1Hz and reaches the maximum inverter output 
% frequency.

% First, we consider voltage and current limits:
Vs_max = Vbatt*eta_inv/sqrt(2);
Iq_max = sqrt(3)*Imax;

% Then, we initially consider Iq=Iq_max util the voltage limit is reached.
% From this point on, we start introducing a negative direct current in 
% order to not exceed this limit for all the frequency range.

eps = 1e-4;
for f = 1:fmax
    omega(f) = 2*pi*f;
    Omega(f) = omega(f)/n;
    
    Iq(f) = Iq_max;
    Id(f) = 0;
    Vq(f) =  Ke*omega(f)+Rs*Iq(f) + omega(f)*Ls*Id(f);
    Vd(f) = -omega(f)*Ls*Iq(f) + Rs*Id(f);
    Vs(f) = sqrt(Vd(f)^2+Vq(f)^2);
    
    while Vs(f) >= Vs_max
        Id(f) = Id(f)-eps;
        Iq(f) = sqrt(Iq_max^2-Id(f)^2);
        Vq(f) =  Ke*omega(f)+Rs*Iq(f) + omega(f)*Ls*Id(f);
        Vd(f) = -omega(f)*Ls*Iq(f) + Rs*Id(f);
        Vs(f) = sqrt(Vd(f)^2+Vq(f)^2);
    end    
    
end

%% Power and Torque calculation
% Here, the power losses are taken into account to evaluate the actual
% power and torque available at the motor shaft and consequently, 
% considering the gear ratio, the ones that are used to drive the scooter.

Pabs = Vd.*Id + Vq.*Iq;         % [VA] Power absorbed by the inverter
Pcu = Rs*(Id.^2 + Iq.^2);       % [W]  Copper power losses
Pfe = Pfe_n.*Omega/wn;          % [W]  Iron power losses
Pmot = Pabs - Pcu - Pfe;        % [W]  Motor power
Pwheel = Pmot*eta_tran;         % [W]  Wheel power
Tmot = Pmot./Omega;             % [Nm] Motor torque
Twheel = Tmot*z*eta_tran;       % [Nm] Wheel torque

% Overall efficiency:
Pin = Pabs/eta_inv;
eta = Pwheel./Pin;

Iq_max = Iq_max/sqrt(3);
Iq = Iq./sqrt(3);
Id = Id./sqrt(3);
%% Results obtained    

figure
grid on; hold on;
plot(omega,Iq_max*ones(size(omega,2),1),'linewidth',2);
plot(omega,Iq,'linewidth',2);
plot(omega,Id,'linewidth',2);
plot(n*[137 137],[-100 100] ,'--k','linewidth',1);
ylabel('Phase Current [A]', 'interpreter', 'latex');
xlabel('$\omega$ [rad/s]', 'interpreter', 'latex')
title('Current Profile', 'interpreter', 'latex')
legend('$I_q^{max}$','$I_q$', '$I_d$','interpreter', 'latex','location','best')

figure
grid on; hold on;
plot(omega,Vs,'linewidth',2);
plot(omega,Vq,'linewidth',2);
plot(omega,Vd,'linewidth',2);
plot(n*[137 137],[-30 50] ,'--k','linewidth',1);
ylabel('Voltage [V]', 'interpreter', 'latex');
xlabel('$\omega$ [rad/s]', 'interpreter', 'latex')
title('Voltage Profile', 'interpreter', 'latex')
legend('$V_s$','$V_q$', '$V_d$','interpreter', 'latex','location','best')

figure
grid on; hold on;
plot(omega,Tmot,'linewidth',2);
plot(n*[139 139],[0 25] ,'--k','linewidth',1);
ylabel('Torque [Nm]', 'interpreter', 'latex');
xlabel('$\omega$ [rad/s]', 'interpreter', 'latex')
title('Motor Torque Profile', 'interpreter', 'latex')

figure
grid on; hold on;
plot(omega,Twheel,'linewidth',2);
plot(n*[139 139],[0 140] ,'--k','linewidth',1);
ylabel('Torque [Nm]', 'interpreter', 'latex');
xlabel('$\omega$ [rad/s]', 'interpreter', 'latex')
title('Wheel Torque Profile', 'interpreter', 'latex')

figure
grid on; hold on;
plot(omega,eta,'linewidth',2);
ylabel('$\eta$', 'interpreter', 'latex');
xlabel('$\omega$ [rad/s]', 'interpreter', 'latex')
title('Overall Efficiency', 'interpreter', 'latex')
