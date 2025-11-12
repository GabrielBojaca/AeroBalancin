clear all; close all; clc;
Wcp = 5.3e-3*9.81; %Peso contrapeso
Wb = 28.7e-3*9.81; % Peso brazo
Wm = (3.5e-3+2.4e-3)*9.81; %Peso motor
d1 = 130.5e-3;
d2 = 50e-3;
e = 13.28e-3;
I = 1.3636e-4;
k =(Wcp*d2-d1*Wm-e*Wb); 
mu = 7.406734e-04;