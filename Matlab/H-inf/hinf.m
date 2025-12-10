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



%%
% Crear sistema en espacio de estados


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


Ka = 8.6928e-5;
Kb = -0.009739;
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


%% 2) DEFINIR PESOS SEGÚN ESPECIFICACIONES
% ------------------------------------------
s = tf('s');

% Banda objetivo: entre 3–10 Hz
%w_bw = 2*pi*20;            
w_bw = 2*pi*2.5;            
%W1: seguimiento razonable (20–40 dB en DC), atenuando en HF
%W1 = makeweight(db2mag(30), w_bw, 0.999);  % decibeles en baja frecuencia
W1 = makeweight(db2mag(40), w_bw, 0.8);  % decibeles en baja frecuencia


% W2: penaliza esfuerzo según el actuador
%W2 = tf(1/10);     
W2 = tf(1/18);                  
% W3: ruido de medición: no tan extremo
%w_pert = 2*pi*50;                     % 50 Hz como antes
%W3 = makeweight(0.99, w_pert, db2mag(20)); %  dB en alta freq 

%w_pert = 2*pi*4;                     % 50 Hz como antes
w_pert = 2*pi*4; 
W3 = makeweight(0.77, w_pert, db2mag(40)); %  dB en alta freq 
%W3 = tf(0)

%=== Normas H-infinito ===
%||S||_∞  = 1.20534
%||T||_∞  = 1.26918
%||KS||_∞ = 10.7485
%w_bw = 2*pi*20;    
%W1 = makeweight(db2mag(30), w_bw, 0.999);  % decibeles en baja frecuencia 
%W2 = tf(1/10); 
%w_pert = 2*pi*50;                     % 50 Hz como antes
%W3 = makeweight(0.99, w_pert, db2mag(20)); %  dB en alta freq 


%=== Normas H-infinito ===
%||S||_∞  = 1.20963
%||T||_∞  = 1.25503
%||KS||_∞ = 12.6662
%w_bw = 2*pi*24;   
%W2 = tf(1/12); 
%w_pert = 2*pi*25;                     % 50 Hz como antes
%W3 = makeweight(0.99, w_pert, db2mag(20)); %  dB en alta freq 


%=== Normas H-infinito ===
%||S||_∞  = 1.198
%||T||_∞  = 1.23447
%||KS||_∞ = 17.559
%w_bw = 2*pi*2.5;    
%W1 = makeweight(db2mag(40), w_bw, 0.8);
%W2 = tf(1/18);   
%w_pert = 2*pi*4; 
%W3 = makeweight(0.77, w_pert, db2mag(40));

%% 3) CONSTRUIR PLANTA GENERALIZADA CON augw
% P_aug = augw(P, W1, W2, W3)

try
    P_aug = augw(P_pwm_deg, W1, W2, W3);
    fprintf('\n=== Planta generalizada P_aug construida (pwm->deg) ===\n');
catch ME
    error('Error al construir P_aug: %s', ME.message);
end

%% 4) SÍNTESIS H-INFINITO CON hinfsyn
% -------------------------------------
% hinfsyn(P_aug, nmeas, nctrl)
% nmeas = número de salidas medidas = 1
% nctrl = número de entradas de control = 1

nmeas = 1;
nctrl = 1;

fprintf('\n=== Iniciando síntesis H-infinito ===\n');
try
    [K, CL, gamma] = hinfsyn(P_aug, nmeas, nctrl);
    fprintf('¡Síntesis exitosa!\n');
    fprintf('Gamma obtenido (norma H-inf óptima): %g\n', gamma);
catch ME
    error('hinfsyn falló: %s\nRevisa dimensiones, estabilidad y pesos.', ME.message);
end

% Nombrar el controlador
K.InputName  = {'angle_error'};
K.OutputName = {'PWM_{Control-Action}'};

fprintf('\n=== Controlador K diseñado ===\n');
K;

%% 5) ANÁLISIS DEL SISTEMA EN LAZO CERRADO
% ------------------------------------------

% Lazo abierto
L = series(P_pwm_deg, K);

% Funciones de sensibilidad
S  = feedback(1, L);     % S = 1/(1+L)
T  = feedback(L, 1);     % T = L/(1+L)
KS = series(K, S);       % KS = K*S

fprintf('\n=== Normas H-infinito ===\n');
try
    normS  = norm(S, inf);
    normT  = norm(T, inf);
    normKS = norm(KS, inf);
    fprintf('||S||_∞  = %g\n', normS);
    fprintf('||T||_∞  = %g\n', normT);
    fprintf('||KS||_∞ = %g\n', normKS);
catch
    warning('No se pudo calcular normas H-inf.');
end

graficar = 1;
if graficar

%% 6) GRÁFICAS DE ANÁLISIS
% --------------------------

omega = logspace(-4, 4, 1000);  

% === Sensibilidad S ===
[magS, ~] = bode(S, omega); magS = squeeze(magS);
% === Sensibilidad complementaria T ===
[magT, ~] = bode(T, omega); magT = squeeze(magT);

% === Pesos 1/W1 y 1/W3 ===
[magW1inv, ~] = bode(inv(W1), omega); magW1inv = squeeze(magW1inv);
[magW3inv, ~] = bode(inv(W3), omega); magW3inv = squeeze(magW3inv);

% === PESO 1/W2 ===
[magW2inv, ~] = bode(inv(W2), omega); magW2inv = squeeze(magW2inv);

% === Esfuerzo KS ===
[magKS, ~] = bode(KS, omega); magKS = squeeze(magKS);

%% --- 6.1) Diagrama Bode de S, T y límites ---
figure('Name','Análisis S y T','NumberTitle','off');

semilogx(omega, 20*log10(magS), 'b', 'LineWidth', 2); hold on;
semilogx(omega, 20*log10(magW1inv), 'b--', 'LineWidth', 1.5);
semilogx(omega, 20*log10(magT), 'r', 'LineWidth', 2);
semilogx(omega, 20*log10(magW3inv), 'r--', 'LineWidth', 1.5);
olive = [0.33 0.42 0.18];
semilogx(omega, 20*log10(magKS), 'Color', olive, 'LineWidth', 2); hold on;
semilogx(omega, 20*log10(magW2inv), '--', 'Color', olive, 'LineWidth', 2);
grid on;
xlabel('Frecuencia (rad/s)');
ylabel('Magnitud (dB)');
title('Sensibilidad S, T, Acción de control KS y pesos inversos');
legend('S','W_1^{-1}','T','W_3^{-1}','KS','W_2^{-1}','Location','Best');
ylim([-45 30])


%% --- 6.2) Esfuerzo de control KS vs 1/W2 ---
figure('Name','Esfuerzo de Control (SISO)','NumberTitle','off');

semilogx(omega, 20*log10(magKS), 'r', 'LineWidth', 2); hold on;
semilogx(omega, 20*log10(magW2inv), 'b--', 'LineWidth', 2);

xline(w_pert, '--m','LineWidth',1.3,'Label','Pert 6 Hz');

grid on;
xlabel('Frecuencia (rad/s)');
ylabel('Magnitud (dB)');
title('Comparación KS vs W_2^{-1}');
legend('|KS|','W_2^{-1}','Location','Best');
xlim([omega(1) omega(end)]);

end


%% 7) RESPUESTA TEMPORAL (STEP)
% -------------------------------
figure('Name','Respuesta al escalón','NumberTitle','off');

% Sistema de referencia -> salida (K*P0 en lazo cerrado)
T_ref = feedback(K*P_pwm_deg, 1);

step(T_ref,5); grid on;
title('Respuesta del ángulo ante un escalón en la referencia');
ylabel('Ángulo (rad)');
xlabel('Tiempo (s)');


%% 8) VERIFICACIÓN DE ESPECIFICACIONES
% --------------------------------------
fprintf('\n=== VERIFICACIÓN DE ESPECIFICACIONES ===\n');

% Ancho de banda: donde |T(jw)| cruza -3 dB
[mag_T, ~] = bode(T, omega);
mag_T = squeeze(mag_T);
mag_T_dB = 20*log10(mag_T);

idx_bw = find(mag_T_dB >= -3, 1, 'last');
if ~isempty(idx_bw)
    bw_achieved = omega(idx_bw)/(2*pi);
    fprintf('Ancho de banda logrado: %.2f Hz %.2f rad/s (requerido: >= %.2f Hz)\n', ...
            bw_achieved,omega(idx_bw),w_bw/(2*pi));
else
    fprintf('Ancho de banda: no se pudo determinar\n');
end

% Normas H∞ ya calculadas arriba:
%fprintf('||KS||_∞ = %.3f\n', normKS);
%fprintf('||S||_∞  = %.3f (menor → mayor robustez)\n', normS);
%fprintf('||T||_∞  = %.3f (menor → mejor rechazo de ruido)\n', normT);

fprintf('\n=== FIN DEL DISEÑO ===\n');

%% 9) DIGITALIZACION
% --------------------------------------

Ts = 0.01;
Kd = c2d(K, Ts);
Ad = Kd.A;
Bd = Kd.B;
Cd = Kd.C;
Dd = Kd.D;
Kd;