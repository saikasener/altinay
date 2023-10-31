clc,clear all, close all;

%% -------------------- Inverse Kinematics --------------------
V = 0.4454; % M/sn
W = 0;   % Rad/sn

r = .105;            % Wheel Radius m
b = .4775;           % AGV Wheel Offset m
GearRatio = 24.685;  % Motor Gear Ratio

WL = (((V - ((W*b)/2)) / r)*180/pi)/6; 
WR = (((V + ((W*b)/2)) / r)*180/pi)/6;

MrRPM = WL * GearRatio;
MlRPM = WR * GearRatio;

%% -------------------- Forward Kinematics --------------------
RmRPM = 1000 * (6*pi/180) / GearRatio; % Rad/Sn
LmRPM = 1000 * (6*pi/180) / GearRatio; % Rad/Sn

Vr = RmRPM * r;
Vl = LmRPM * r;

Vrobot = (Vr + Vl) / 2
Wrobot = (-Vr + Vl) / b

% -------------------------------------------------------------
















