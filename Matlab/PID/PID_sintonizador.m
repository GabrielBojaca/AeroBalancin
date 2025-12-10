clear all; close all; clc;
Wcp = 5.3e-3*9.81; %Peso contrapeso
Wb = 28.7e-3*9.81; % Peso brazo
Wm = (3.5e-3+2.4e-3)*9.81; %Peso motor
d1 = 130.5e-3;
d2 = 50e-3;
e = 13.28e-3;
J = 1.3636e-4;
I=J;
Y = (Wcp*d2-d1*Wm-e*Wb); 
k = Y;
B = 4e-4;
mu = B;

s = tf('s');

% --------------- Modelo Lineal -------------------------

theta0_deg = 80;         % punto de linearización en grados
theta0 = deg2rad(theta0_deg);

a1 = B / J;
a0 = (Y / J) * cos(theta0);

A = [0, 1;
     -a0, -a1];
Bmat = [0; 1/J];
C = [1, 0];
D = 0;

% FT Numerica
num = 1/J;
den = [1, B/J, (Y/J)*cos(theta0)];
G_tf = tf(num, den);
G_tf;

%% --- Actuador y escala salida ---
Ka = 8.6928e-5;  
scale_deg = 180/pi;

P0 = ss(A, Bmat, C, D);
P0.InputName  = {'fm'};
P0.OutputName = {'angulo_rad'};

m =   1.2984e-04;
b =  -0.0231;

Ka = m;
Kb = b;
scale = 180/pi;

Act = tf(Ka);
Scale = tf(scale);

P_pwm_deg = series(Scale * P0, Act);
P_pwm_deg.InputName = 'pwm';
P_pwm_deg.OutputName = 'angle_deg';

fprintf('=== Planta desde PWM a angulo (deg) ===\n');
P_pwm_deg;

G  = tf(P_pwm_deg);
G

%% ----------------- CONTROLADOR EN TÉRMINOS DE τi y τd --------------------

% Escoge tiempos (AJUSTA)
tau_i = 0.65;     % tiempo integral
tau_d = 0.15;    % tiempo derivativo

% Controlador base sin ganancia:
% C0(s) = ( 1 + 1/(tau_i s) + tau_d s )
C0 = (1 + 1/(tau_i*s) + tau_d*s);

% Lazo abierto sin ganancia
L = C0 * G;

figure;
rlocus(L); grid on;
title('Root Locus en términos de tau_i y tau_d');

% Seleccionar ganancia K (AJUSTA)
K = 2.3;

% Convertir a Kp, Ki, Kd
Kp = K;
Ki = K / tau_i;
Kd = K * tau_d;

fprintf("\nPID usando tiempos τ:\nKp=%.4f\nKi=%.4f\nKd=%.4f\n",Kp,Ki,Kd);

C = pid(Kp,Ki,Kd);

% Respuesta al escalón
T = feedback(C*G,1);

figure;
step(T); grid on;
title('Respuesta al escalón con PID en términos de τ_i y τ_d');
