%% Parámetros del aerobalancín
Wcp = 5.3e-3 * 9.81;
Wb  = 28.7e-3 * 9.81;
Wm  = (3.5e-3 + 2.4e-3) * 9.81;

d1 = 130.5e-3;
d2 = 50e-3;
er = 13.28e-3;

I  = 1.3636e-4;
mu = 4.406734e-04;

k = (Wcp*d2 - d1*Wm - er*Wb);
b = d1 / I;

%% Parámetros del SMC
lambda = 1.5;
eta = 0.1;   % margen de robustez

%% Rango de operación
ang_deg = linspace(0, 120, 200);
ang_rad = ang_deg * pi/180;
wz = linspace(-5, 5, 200);

[A, W] = meshgrid(ang_rad, wz);

%% Dinámica nominal f(x)
fx = (k/I).*sin(A) - (mu/I).*W;

%% Control equivalente
ueq = -(fx + lambda.*W) / b;

%% Cálculo de beta mínimo
beta_min = max(abs(ueq(:))) + eta;

fprintf("Valor teórico de beta_min = %.5f\n", beta_min);