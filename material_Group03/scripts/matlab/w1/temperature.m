% TITLE: temperature.m
% IMPLEMENTED BY: Pol Rius (423483)
% DESCRIPTION: estimates the thermal resistance R from temperature
% measurements

clear all; clear figure; close all;

%% Loading data

% Contants
qh = 0.1; % [W]
C = 0.25; % [J/K]

% Cols are: time, sensor_id, sensor_x, sensor_y, temp_in, temp_out
M = readmatrix('temperature.csv');

times = M(:,1);
temperatures_in = M(:,5); 
temperatures_out = M(:,6);


%% Calculation

R =100; % [°C/W]
Rtol = 1e-3;


while true
    Y = temperatures_in-temperatures_out;
    X = 1-exp(-times/(R*C));
    
    c = X\Y;
    Rnew = c/qh;

    if abs(Rnew-R) < Rtol
        R = Rnew;
        break;
    else
        R = Rnew;
    end
end

R
mean(temperatures_out)

%% Visualization
npts = 100;
xeval = linspace(1, max(times), npts);
yfit = mean(temperatures_out) + qh*R*(1-exp(-xeval/(R*C)));

figure(1);
hold on;
plot(times, temperatures_in, 'x');
plot(xeval, yfit);
xlabel("Time [s]")
ylabel("Inside temperature [degree Celsius]")
title("Inside temperature fit and plot")

