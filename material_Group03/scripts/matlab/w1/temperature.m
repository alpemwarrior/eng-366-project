% TITLE: temperature.m
% IMPLEMENTED BY: Pol Rius (423483)
% DESCRIPTION: estimates the thermal resistance R from temperature
% measurements

clear all; clear figure; close all;

%% Calculation

% Contants
qh = 0.1; % [W]
C = 0.25; % [J/K]

% Cols are: time, sensor_id, sensor_x, sensor_y, temp_in, temp_out
M = readmatrix('temperature.csv');

times = M(:,1);
temperatures_in = M(:,5); 
temperatures_out = M(:,6);

% From this point onwards the system is considered to be in Steady-State
% Find index corresponding to this time
t_min = 200;
[minm, I] = min(abs(times-t_min));


slice = temperatures_in-temperatures_out;
slice = slice(I:end);
ssv = mean(slice);

% Steady-state value is qh*R
R = ssv/qh

%% Visualization
%plot(times, temperatures_in, 'x');
%xline(t_min);
%yline(R*qh+T_o(1))



