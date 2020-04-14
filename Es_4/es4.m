clear all; close all; clc; 

%%  Data 

Vn = 110;               % [V]       Nominal voltage
Tn = 9.6;               % [Nm]      Nominal torque
En = 105;               % [V]       Back e.m.f
wn = 3000*pi/30;        % [rad/s]   Nominal speed
In = 30;                % [A]       Nominal current
Kt = 0.32;              % [Nm/A]    Torque constant
R = 0.32;               % [Ohm]     Motor resistance
L = 3e-3;               % [H]       Motor inductance
J = 0.0013;             % [kg*m^2]  Motor inertia
wp_i = 500;             % [rad/s]   Cut-off frequency PI current regulator
wp_s = 100;             % [rad/s]   Cut-off frequency PI speed regulator
phiM_i = 80*pi/180;     % [rad]     Phase margin current regulator
phiM_s = 60*pi/180;     % [rad]     Phase margin speed regulator
fc = 8000;              % [Hz]      Chopper switching frequency
Tsi = 0.25e-3;          % [s]       Sampling time current transducer
Tsw = 1e-3;             % [s]       Sampling time speed transducer

%% Ideal system
% First, the power block transfer function is considered as equal to 1 and
% no sampling effects are taken into account.

% Motor TF
syms w
s = tf('s');
B = 1/(R+L*s);
% Current Regulator
B_syms = poly2sym(cell2mat(B.Num),w)/poly2sym(cell2mat(B.Den),w);
B1=subs(B_syms,w,wp_i*1i);
kp_iID = double(1/abs(B1)*cos(-pi+phiM_i-angle(B1)));
ki_iID = double(-wp_i/abs(B1)*sin(-pi+phiM_i-angle(B1)));

G = (kp_iID+ki_iID/s)*B;
L = feedback(G , 1);

[Gmi_ID,Pmi_ID]=margin(G);
figure()
asymp(G)
title('Bode diagram of current loop (ideal system)')

% Speed Regulator
Bw = L/s/J;
Bw_syms = poly2sym(cell2mat(Bw.Num),w)/poly2sym(cell2mat(Bw.Den),w);
B2=subs(Bw_syms,w,wp_s*1i);
kp_sID = double(1/abs(B2)*cos(-pi+phiM_s-angle(B2)));
ki_sID = double(-wp_s/abs(B2)*sin(-pi+phiM_s-angle(B2)));
Gw = (kp_sID+ki_sID/s)*Bw;

[Gmw_ID,Pmw_ID]=margin(Gw);
figure()
asymp(Gw)
title('Bode diagram of speed loop (ideal system)')

%% Non linearity effects
% These effects are related to the electronic converter and to the current
% and speed transducers.

% Chopper
g = Vn/1;
Trm = 1/2/fc;
C = g/(1+Trm*s);

% Transducers
Hi = 1/In;
Hw = 1/wn;
Ti = 1/(1+s*Tsi);
Tw = 1/(1+s*Tsw);

% Current Regulator
Bi = B*C*Hi*Ti;
B_syms = poly2sym(cell2mat(Bi.Num),w)/poly2sym(cell2mat(Bi.Den),w);
B1=subs(B_syms,w,wp_i*1i);
kp_i = double(1/abs(B1)*cos(-pi+phiM_i-angle(B1)));
ki_i = double(-wp_i/abs(B1)*sin(-pi+phiM_i-angle(B1)));
Gi = (kp_i+ki_i/s)*Bi;
%Li = 1/Hi*Gi/(1+Gi);
Li = feedback(Gi / Hi, Hi);

[Gmi,Pmi]=margin(Gi);
figure()
asymp(Gi)
title('Bode diagram of current loop')

% Speed Regulator
B1w = Li/s/J*Hw*Tw;
B1w_syms = poly2sym(cell2mat(B1w.Num),w)/poly2sym(cell2mat(B1w.Den),w);
B3=subs(B1w_syms,w,wp_s*1i);
kp_s = double(1/abs(B3)*cos(-pi+phiM_s-angle(B3)));
ki_s = double(-wp_s/abs(B3)*sin(-pi+phiM_s-angle(B3)));
G1w = (kp_s+ki_s/s)*B1w;

[Gmw,Pmw]=margin(G1w);
figure()
asymp(G1w)
title('Bode diagram of speed loop')

%% Gain results

fprintf('\n\n------------------------------------------\n\n\t')
fprintf('Results obtained IDEAL system:\n\n\t')
fprintf('Current loop: Kp = %1.2f   Ki = %3.1f\n\t',kp_iID,ki_iID)
fprintf('Speed loop:   Kp = %1.2f   Ki = %1.2f\n\n\t',kp_sID, ki_sID)
fprintf('Results obtained REAL system:\n\n\t')
fprintf('Current loop: Kp = %1.2f   Ki = %2.1f\n\t',kp_i, ki_i)
fprintf('Speed loop:   Kp = %1.2f   Ki = %2.1f\n\t',kp_s, ki_s)
fprintf('\n------------------------------------------\n\n')


%% DIGITAL QUANTIZATION 
% In order to further increase the level of detail of the non ideality of
% the model, we need to consider also the quantization of the two
% transducers and of the electronic converter. 

bit_I = 12;
bit_PWM = 10;
ppr_speed = 2000;

t_f = 30;

t = sim('es_4.slx', t_f);

figure()
plot(t, Om_input.Data(:), '--r');
hold on;
plot(t, Om_output.Data(:), 'b');
title('Step response')
