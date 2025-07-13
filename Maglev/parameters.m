close all;


%% Analog PID gains
% Based on the provided gain, calculates the SPI command, that should be
% transfered toward the digital potentiometers

% The resistance value of the digital potentiometer, and the resistance of
% the wiper
R_P_dig = 1e5;
R_I_dig = 1e5;
R_D_dig = 1e5;
Rw = 52;
R_prot = 4700;
R_fb = 20000;

R_P_fb = 47000;
R_P = 10000;
R_I = 10000;
R_D = 300000;
C_I = 0.1e-6;
C_D = 0.2e-6;

% Gain provided by each operational-amplifier
A_P = - R_P_fb / R_P;
A_I = - 1 / (R_I * C_I);
A_D = - R_D * C_D;

K_P_max = -(R_fb/(Rw + R_prot)) * A_P
K_P_min = -(R_fb/(Rw + R_prot + R_P_dig)) * A_P

K_I_max = -(R_fb/(Rw + R_prot)) * A_I
K_I_min = -(R_fb/(Rw + R_prot + R_I_dig)) * A_I

K_D_max = -(R_fb/(Rw + R_prot)) * A_D
K_D_min = -(R_fb/(Rw + R_prot + R_D_dig)) * A_D

K_P = 15.01;
K_I = 230.92;
K_D = 0.24016;

R_P_dig_act = (R_fb * abs(A_P))/K_P - Rw - R_prot
R_I_dig_act = (R_fb * abs(A_I))/K_I - Rw - R_prot
R_D_dig_act = (R_fb * abs(A_D))/K_D - Rw - R_prot


% To get the resistance's real value
cmd_P_dec = round((R_P_dig_act / R_P_dig) * 256 + 0.49);
cmd_I_dec = round((R_I_dig_act / R_I_dig) * 256 + 0.49);
cmd_D_dec = round((R_D_dig_act / R_D_dig) * 256 + 0.49);

% Transforming the command, regarding to wiper position, to hexadecimal
% values
cmd_P = dec2hex(cmd_P_dec);
cmd_I = dec2hex(cmd_I_dec);
cmd_D = dec2hex(cmd_D_dec);

R_P_dig_real = (R_P_dig * cmd_P_dec)/256 + Rw;
R_I_dig_real = (R_P_dig * cmd_I_dec)/256 + Rw;
R_D_dig_real = (R_P_dig * cmd_D_dec)/256 + Rw;

% The real gains, obtained for that commands
K_P_real = abs(A_P) * R_fb / (R_P_dig_real + Rw + R_prot)
K_I_real = abs(A_I) * R_fb / (R_I_dig_real + Rw + R_prot)
K_D_real = abs(A_D) * R_fb / (R_D_dig_real + Rw + R_prot)


%% PID parameters, Ziegler-Nichols and Tyreus-Luyben 

Ku = 31;
Pu = 0.105;

% Ziegler-Nichols PI
Kp_PI_zig = 0.45 * Ku
Ti_PI_zig = Pu/1.2

% Ziegler-Nichols PID
Kp_PID_zig = 0.6 * Ku
Ti_PID_zig = Pu / 2
Td_PID_zig = Pu / 8

% Modified Ziegler-Nichols PID less overshout
Kp_PID_zig_modif_os = 0.33 * Ku
Ti_PID_zig_modif_os = Pu / 2
Td_PID_zig_modif_os = Pu / 3

% Modified Ziegler-Nichols PID no overshoot
Kp_PID_zig_modif = 0.2 * Ku
Ti_PID_zig_modif = Pu / 2
Td_PID_zig_modif = Pu / 3

% Tyreus-Luyben PI
Kp_PI_tyr = Ku / 3.2;
Ti_PI_tyr = 2.2 * Pu;

% Tyreus-Luyben PID
Kp_PID_tyr = Ku / 3.2;
Ti_PID_tyr = 2.2 * Pu;
Td_PID_tyr = Pu / 6.3;




%% Linearization
syms x v i kv R L ga m u km r;
f1 = v;
f2 = ga - (kv/m)*(i/x)^2;
f3 = -(R/L)*i - (km/L)*((x*v)/(r^2 + x^2)^(5/2)) + u/L;  
% state variables are x, v, i and the input is u

A_sym(1,:) = [diff(f1,x) diff(f1,v) diff(f1,i)];
A_sym(2,:) = [diff(f2,x) diff(f2,v) diff(f2,i)];
A_sym(3,:) = [diff(f3,x) diff(f3,v) diff(f3,i)];

B_sym = [diff(f1,u); diff(f2,u); diff(f3,u)];


% Model parameter gen
m = 0.01;           % [kg] mass of the neodymium magnet
g = 9.81; 
L = 0.015904;          % [H] coil inductance
R = 14.4;             % [ohm] coil resistance
r = 0.0125;         % [m] coil radius
l = 0.02;           % [m] coil length
Area = pi*r^2;         % [m2] coil surface facing the magnet

mu = 4*pi*1e-7;      % permeability of free space
N = sqrt(L*l/(mu*Area));  % [-] number of turns of the coil
km = 3/2*mu*m*N*r^2; % s

% Equilibrium points
u0 = 6;     % voltage proportional to 50% duty cycle
x0 = 0.0151; % equilibrium position, Hall voltage in this point 1.79 V, at a duty cycle of 50%
v0 = 0;
i0 = 0.417;

syms kv
eqn = g-(kv/m)*(i0^2/x0^2) == 0;
[A,B] = equationsToMatrix([eqn], [kv])
X = linsolve(A,B);
kv = double(X)

a21 = (2*kv*i0^2)/(x0^3*m);
a23 = -(2*i0*kv)/(m*x0^2);
a31 = 0;    % since the velocity in the equilibrium state will be 0 m/s
a32 = -(km*x0)/(L*(r^2 + x0^2)^(5/2));
a33 = - R/L;

A = [0 1 0; a21 0 a23; a31 a32 a33];
B = [0; 0; 1/L];
C = [1 0 0];
D = 0;

[num, den] = ss2tf(A, B, C, D);
Hf = tf(num, den);


rlocus(Hf);



%% Inductance measurement

% Experimental data
freq = [4.649e3, 9.041e3, 18.099e3, 40.5e3];     % Frequency in Hz
cap = [68e-9, 22e-9, 10e-9, 1e-9];               % Capacitance in Farads

% Linearized variables
freq2 = freq.^2;           % Square of frequency
cap1 = 1 ./ cap;           % Inverse of capacitance

% Linear regression to find gradient
p = polyfit(cap1, freq2, 1);
slope = p(1);              % a = slope = 1/(4π²L)

% Compute inductance
L = 1 / (4 * pi^2 * slope);

% Display results
fprintf('Gradient (slope a) = %.3e\n', slope);
fprintf('Calculated Inductance L = %.6f H\n', L);

% Generate fitted values for plotting
x_fit = linspace(min(cap1), max(cap1), 100);
y_fit = polyval(p, x_fit);

% Plot
plot(cap1, freq2, 'bo', 'MarkerFaceColor', 'b'); hold on;
plot(x_fit, y_fit, 'r--', 'LineWidth', 1.5);
xlabel('1/C [1/F]');
ylabel('f^2 [Hz^2]');

x_margin = 0.2 * (max(cap1) - min(cap1));
y_margin = 0.2 * (max(freq2) - min(freq2));

% Apply custom axis limits
axis([min(cap1)-x_margin, max(cap1)+x_margin, min(freq2)-y_margin, max(freq2)+y_margin]);
legend('Measured data', sprintf('Fit: y = %.2e·x + %.2e', p(1), p(2)));
grid on;
