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
%fprintf('\nFunción de transferencia numérica G(s):\n');
%disp(G_tf);
G_tf;

%% --- Definir actuador (pwm -> fm) y escala salida (rad->deg) ---
Ka = 8.6928e-5;     % fm = Ka * pwm + Kb  (ganancia)
%Kb = -0.009739;     % offset ??
scale_deg = 180/pi; % convertir rad -> grados

% Planta original P0: input 'fm' (fuerza) -> salida angulo (rad)
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

% --- Verificar polos ---
poles = pole(G);
disp('Polos de G(s):');
disp(poles);

% Si hay polos inestables, avisar
if any(real(poles) > 0)
    warning('La planta es inestable: se sintonizará PID para estabilizar.');
end

% --- Frecuencia objetivo de cruce ---
% Escogemos una frecuencia donde queremos que el sistema sea estable y rápido.
% Puedes cambiar esta w_c si deseas otro comportamiento.
w_c = 20;   % [rad/s] objetivo razonable para plantas de segundo orden inestables El Hinf da 23.9

% --- Sintonizar PID ---
[C, info] = pidtune(G, 'PID', w_c);

disp('Controlador PID obtenido:');
C

disp('Información de sintonía:');
info

% --- Sistema compensado ---
T = feedback(C*G, 1);

% --- Gráficas ---
figure;
step(T, 5);
grid on;
title('Respuesta al Escalón del Sistema Compensado');

figure;
margin(C*G);
title('Diagrama de Bode con Márgenes de Estabilidad');
grid on;


tau_i = C.Kp/C.Ki
tau_d = C.Kd/C.Kp