clear all; close all; clc;
Wcp = 5.3e-3*9.81; %Peso contrapeso
Wb = 28.7e-3*9.81; % Peso brazo
Wm = (3.5e-3+2.4e-3)*9.81; %Peso motor
d1 = 130.5e-3;
d2 = 50e-3;
e = 13.28e-3;
I = 1.3636e-4;
k =(Wcp*d2-d1*Wm-e*Wb); 

%angulos = linspace(0,-85,18)';
%angulos = deg2rad(90 + angulos);

% Nueva tabla
%  REF	PWM
%  10	230
%  20	350
%  30	435
%  40	510
%  50	575
%  60	630
%  70	650
%  80	680
%  90	692


%pwm = [871,868,862,849,826,800,772,737,700,663,618,570,510,453,380,305,212,-100]';
pwm = [692,680,650,630,575,510,435,350,230]'; %Ultima calibracion

angulos = linspace(90,10,9)';
angulos = deg2rad(angulos);
theta = angulos;
%

Fm = -(sin(theta)*(Wcp*d2-d1*Wm-e*Wb))/d1;


voltaje = pwm*(4.4235/1023);


angulos = rad2deg(angulos);
T = table(pwm, voltaje, angulos, Fm*1000, 'VariableNames',{'PWM','Voltaje (V)', 'Angulo (°)', 'Fm (mN)'});

% Mostrar tabla
disp(T);



figure;
uitable('Data', T{:,:}, ...
        'ColumnName', T.Properties.VariableNames, ...
        'Units', 'normalized', ...
        'Position', [0.1 0.1 0.8 0.8]);

figure

plot(voltaje,Fm)
xlabel('Voltaje (V)')
ylabel('Fuerza (N)')

%%

syms theta(t)


% --- Parámetros ---
I;    % [kg·m²]
%mu = 7.406734e-04% N·m·s/rad     % [N·m·s/rad]
mu = 2e-04
k =(Wcp*d2-d1*Wm-e*Wb);       % [N·m/rad]



% --- Ecuación diferencial ---
eqn = I*diff(theta,t,2) + mu*diff(theta,t) - k*theta == 0;

% --- Condiciones iniciales ---
Dtheta = diff(theta,t);
conds = [theta(0) == deg2rad(17), Dtheta(0) == 0];

% --- Resolver ---
sol = dsolve(eqn, conds);
theta_t = simplify(sol);
disp('θ(t) =');

% --- Convertir a función numérica ---
theta_fun = matlabFunction(theta_t);

% --- Graficar ---
t = linspace(0,10,1000);
plot(t, theta_fun(t),'LineWidth',2);
xlabel('Tiempo [s]');
ylabel('\theta(t)');
title('Respuesta \theta(t)');
grid on;

% --- Frecuencias ---
wn = sqrt(k/I);
wd = sqrt(wn^2 - (mu/(2*I))^2);
fprintf('Frecuencia natural wn = %.3f rad/s\n', wn);
fprintf('Frecuencia amortiguada wd = %.3f rad/s\n', wd);


vpa(theta_t,4)


%%


angulo_exp =[17.14,7.56,-9.93,-3.95,9.41,3.52,-6.59,-1.23,5.81,1.15,-3.33,0,2.64,0.36,-0.61,0.27];
angulo_exp = deg2rad(angulo_exp);
tiempo_exp = linspace(0,length(angulo_exp)*0.2,16);
hold on;
plot(tiempo_exp,angulo_exp);


%%
%%
%%
%%
%close all
figure;

% Omitir el primer dato
pwm2 = pwm(1:end-1);
Fm2  = Fm(1:end-1);


plot(pwm2, Fm2, 'o'); hold on;
xlabel("PWM");
ylabel("Fm (N)");
grid on;

% --- REGRESIÓN LINEAL ---
p = polyfit(pwm2, Fm2, 1);   % Ajuste lineal Fm(pwm)

% Coeficientes
m = p(1);
b = p(2);

% Valores ajustados para los puntos medidos
Fm_pred = polyval(p, pwm2);

% Cálculo R^2
SS_res = sum((Fm2 - Fm_pred).^2);
SS_tot = sum((Fm2 - mean(Fm2)).^2);
R2 = 1 - SS_res / SS_tot;

fprintf("R^2 = %.6f\n", R2);

% Curva suave
pwm_fit = linspace(min(pwm2), 1023, 300);
Fm_fit = polyval(p, pwm_fit);

% Graficar curva ajustada
plot(pwm_fit, Fm_fit, 'LineWidth', 2);

% Mostrar ecuación
fprintf("Fm(PWM) = %.6f * PWM + %.6f\n", m, b);
