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
     -a0, -a1]
B = [0; 1/J]
C = [1, 0];
D = 0;

% --- Parámetros del Modelo de Referencia (Cómo queremos que se comporte) ---
% Queremos que sea rápido pero sin sobrepaso (Amortiguamiento crítico)
wn = 3;       % Frecuencia natural (velocidad de respuesta)
zeta = 1;     % Factor de amortiguamiento (1 = sin oscilación)

Am = [0, 1; 
      -wn^2, -2*zeta*wn]
Bm = [0; 
      wn^2]

% --- Configuración de la Simulación ---
T_sim = 10;          % Tiempo total (segundos)
dt = 0.001;          % Paso de tiempo (1 ms)
time = 0:dt:T_sim;   % Vector de tiempo
ref_val = 80;        % REFERENCIA DE 80 GRADOS

%% 2. CÁLCULO DE GANANCIAS IDEALES (LA SOLUCIÓN A TU PROBLEMA)
% Condición de coincidencia de modelo:
% Kx_ideal resuelve: A + B*Kx = Am
% Kr_ideal resuelve: B*Kr = Bm

% Usamos la pseudoinversa (pinv) por si B no es cuadrada
Kx_ideal = pinv(B) * (Am - A); 
Kr_ideal = pinv(B) * Bm;

fprintf('Ganancias Ideales Calculadas:\n');
fprintf('Kp (Posición): %.4f\n', Kx_ideal(1));
fprintf('Kv (Velocidad): %.4f\n', Kx_ideal(2));
fprintf('Kr (Feedforward): %.4f\n', Kr_ideal);

%% 3. CONFIGURACIÓN DEL CONTROL ADAPTATIVO
Gamma_x = 0.000001;    % Tasa de aprendizaje para Kx 
Gamma_r = 0.000001;    % Tasa de aprendizaje para Kr

% Ecuación de Lyapunov (P) para la ley de adaptación
Q = eye(2) * 10;        % Matriz de peso Q
P = lyap(Am', Q);       % Resuelve Am'*P + P*Am = -Q

%% INICIALIZACIÓN DE VARIABLES
x = [0; 0];     
xm = [0; 0];    
r = 0;           

% la solución perfecta.
Kx = Kx_ideal;   
Kr = Kr_ideal;

% Guardar datos para graficar
history_x = zeros(2, length(time));
history_xm = zeros(2, length(time));
history_Kx = zeros(2, length(time));
history_u = zeros(1, length(time));

%% 5. BUCLE DE SIMULACIÓN
for i = 1:length(time)
    t = time(i);
    
    % Cambio de referencia (Escalón a los 1s)
    if t > 1
        r = ref_val;
    else
        r = 0;
    end

    if t > 6 && t < 6 + dt
        x(1) = x(1) + 30; % Simulamos que lo movió 30 grados de golpe
    end
   
    e = x - xm; % Error de seguimiento
    
    % --- b) Ley de Adaptación (Lyapunov) ---
    % Derivada de las ganancias
    % dKx = -Gamma * B' * P * e * x'
    error_weighted = (B' * P * e); 
    
    dKx = -Gamma_x * error_weighted * x'; 
    dKr = -Gamma_r * error_weighted * r;
    
    % Integración de las ganancias (Euler)
    Kx = Kx + dKx * dt;
    Kr = Kr + dKr * dt;
    
    % ROBUSTEZ

    % Para que A_cl = A + B*Kx sea estable, Kx suelen ser negativos.
    
    if Kx(1) > 0  % Evita que la ganancia de posición cambie de signo
       Kx(1) = 0;
    end
    
    % Ley de Control
    u = Kx * x + Kr * r;
    
    % Evolución del Sistema (Planta y Modelo) 
    dx = A*x + B*u;
    dxm = Am*xm + Bm*r;
    
    x = x + dx * dt;
    xm = xm + dxm * dt;
    
    % Guardar datos
    history_x(:, i) = x;
    history_xm(:, i) = xm;
    history_Kx(:, i) = Kx';
    history_u(i) = u;
end

%% 6. GRAFICAR RESULTADOS
figure('Color', 'black');

subplot(3,1,1);
plot(time, history_x(1,:), 'LineWidth', 2); hold on;
plot(time, history_xm(1,:), 'r--', 'LineWidth', 2);
title('Posición: Real vs Modelo Referencia');
ylabel('Grados'); grid on; legend('Real', 'Modelo');

subplot(3,1,2);
plot(time, history_u, 'g');
title('Esfuerzo de Control (u)');
grid on;

subplot(3,1,3);
plot(time, history_Kx(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(time, history_Kx(2,:), 'Color', [0.9 0.5 0]);
yline(Kx_ideal(1), 'b--'); 
yline(Kx_ideal(2), '--', 'Color', [0.9 0.5 0]);
title('Adaptación de Ganancias Kx');
legend('K pos', 'K vel', 'Ideal Pos', 'Ideal Vel');
grid on; xlabel('Tiempo (s)');