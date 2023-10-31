clc, clear, close all, warning off;
%% Hamal AGV Lifting Mechanism Simulation
% Written By: Rasit EVDUZEN
% Company: ALTINAY ROBOTIC
% 22-Sep-2022


% Parameters
GForce = 9.80665;
Payload = 750 + 50;   % Payload + LiftMechanism Load
FinalStroke = 100;        % Lifting Mechanism Stroke 
InitialStroke = 0; 

SimTime = 5;           % Lifting Time
Ts = 1e-3;             % Sampling Time

Tblend = SimTime/3;
Ta = SimTime - (2 * Tblend);
LinearVel = FinalStroke / (Tblend + Ta);
[p,pd,pdd] = LSPBTrajectoryGeneration(InitialStroke, FinalStroke, SimTime, LinearVel, Ts);
TimeVec = (linspace(0,SimTime,length(p)))';
StaticForce = Payload * GForce;
DynamicForce = pdd * Payload;
TotalForce = StaticForce + DynamicForce;
TotalPower = (pd .* 1e-3) .* TotalForce; 

LeadScrewPitch = 5;
GearBoxTurn = FinalStroke / LeadScrewPitch;
GearRatio = 5; 
MotorTurn = GearBoxTurn * GearRatio;
InitialAngle = 0;
FinalAngle = 360 * MotorTurn;
AngularVel = FinalAngle / (Tblend + Ta);

[Mp,Mpd,Mpdd] = LSPBTrajectoryGeneration(InitialAngle, FinalAngle, SimTime, AngularVel, Ts);

%% Plot Data
fh = figure();
set(gcf,'Position',[0 0 1720 900],'Color','white')


subplot(321)
plot(TimeVec,p,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$X(t) \ [mm]$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Linear Position','fontsize',12)

subplot(322)
plot(TimeVec,TotalForce,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$F(t) \ [N]$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Force','fontsize',12)

subplot(323)
plot(TimeVec,pd,'r','LineWidth',2), grid on, hold on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$\dot{X}(t) \ [mm/sn]$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Linear Velocity ','fontsize',12)

subplot(324)
plot(TimeVec,TotalPower,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$P(t) \ [Watt]$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Power','fontsize',12)


subplot(325)
plot(TimeVec,pdd,'r','LineWidth',2), grid on,hold on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$\ddot{X}(t) \ [mm/sn^2]$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Linear Acceleration','fontsize',12)

% ---------- Motor Trapezoidal Profile ---------
fh = figure();
set(gcf,'Position',[0 0 1720 900],'Color','white')

subplot(221)
plot(TimeVec,Mp,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$\theta(t) (Deg)$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Angular Position','fontsize',12)

subplot(222)
plot(TimeVec,Mpd,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$\dot{\theta}(t) (Deg/Sn)$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Angular Velocity','fontsize',12)

subplot(223)
plot(TimeVec,Mpd/6,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$\dot{\theta}(t) (Rpm)$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Angular Velocity (Rpm)','fontsize',12)

subplot(224)
plot(TimeVec,Mpdd/6,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('$\ddot{\theta}(t) (Rpm/Sn)$','Interpreter','latex','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Angular Acceleration (Rpm/Sn)','fontsize',12)



fh = figure();
set(gcf,'Position',[0 0 1720 900],'Color','white')

EncoderRes = 4096; % PPR
plot(TimeVec,(Mp/360)*EncoderRes,'r','LineWidth',2), grid on
axis auto
xlabel("Time",'FontSize',12,'FontWeight','bold','Color','k')
ylabel('Incremental Encoder','fontsize',15)
xline(Tblend,'k:','LineWidth',2)
xline(SimTime-Tblend,'k:','LineWidth',2)
title('Angular Position Inc','fontsize',12)
