clc,clear all,close all
% 10-Jan-2023
% Motor Value Plot
% -----------------------------------------------
% Timestamp [µs]	
% Statusword	
% Position actual value [Inc]	
% Position demand internal value [Inc]	
% Velocity actual value [RPM]	
% Velocity demand value [RPM]	
% Torque actual value [‰]	
% Torque demand [‰]	
% Position following error [Inc]	
% Velocity following error	
% Torque following error [‰]
% -----------------------------------------------

load MotorData

LeftMotorTorque = Left(:,7)/10;
RightMotorTorque = Right(:,7)/10;
time = Left(:,1)/1e6;


%% Plot Torque
subplot(2,2,[1 2])
plot(time,LeftMotorTorque,'r'),grid on, hold on
plot(time,RightMotorTorque,'b--')
xlabel('Time [Sn]'),ylabel('% of Nominal Torque')
legend("Left Motor","Right Motor")

subplot(2,2,[3 4])
plot(time,(LeftMotorTorque - RightMotorTorque),'k--'),grid on, hold on
xlabel('Time [Sn]'),ylabel('% of Nominal Torque')
legend("Torque Difference")

%% Plot Velocity
figure
subplot(2,2,[1 2])
plot(time,Left(:,5),'r'),grid on, hold on
plot(time,Left(:,6),'b--')
xlabel('Time [Sn]'),ylabel('Angular Vel [Rpm]')
legend("Actual","Desired")
title("Left Motor")

subplot(2,2,[3 4])
plot(time,Right(:,5),'r'),grid on, hold on
plot(time,Right(:,6),'b--')
xlabel('Time [Sn]'),ylabel('Angular Vel [Rpm]')
legend("Actual","Desired")
title("Right Motor")

