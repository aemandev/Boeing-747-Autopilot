close all; clear all; clc

%{
% % Going to Investigate Condition 5 -> 20,000 ft at M=0.5
% % % g = 32.17405; % Gravity ft/s*s
% % % % % % % % % % % M = 0.5;
% % % % % % % % % % % a = 1036.7867; % Speed of sound at 20000 ft, ft/s
% % % % % % % % % % % U1 = M*a;
% % % % % % % % % % % rho = 0.001267; % slug/ft^3
% % % % % % % % % % % qbar = .5*rho*U1^2; % Dyanmic Pressure
% % % % % % % % % % % S = 5500; % ft^2 % Wing area
% % % % % % % % % % % cBar = 27.3; % cord width
% % % % % % % % % % % W = 636636; % lb
% % % % % % % % % % % m = W/g;
% % % % % % % % % % % Iy = 33.1*10^6; % slug-ft^2
% % % % % % % % % % % 
% % % % % % % % % % % 
% % % % % % % % % % % Cl = .680;
% % % % % % % % % % % Cd = 0.0393;
% % % % % % % % % % % 
% % % % % % % % % % % Cla = 4.67;
% % % % % % % % % % % Cda = 0.366;
% % % % % % % % % % % Cma = -1.146;
% % % % % % % % % % % ClaDot = 6.53;
% % % % % % % % % % % CmaDot = -3.35;
% % % % % % % % % % % theta0 = 0;
% % % % % % % % % % % Clq = 5.13;
% % % % % % % % % % % Clu = 0;
% % % % % % % % % % % Clde = 0.356;
% % % % % % % % % % % Cmq = -20.7;
% % % % % % % % % % % CLM = -0.0875;
% % % % % % % % % % % CmM = -.121;
% % % % % % % % % % % CMu = 0;
% % % % % % % % % % % Cma = -1.146;
% % % % % % % % % % % Cdu = 0;
% % % % % % % % % % % Cda = 0.366;
% % % % % % % % % % % Cmde = -1.43;
% % % % % % % % % % % 
% % % % % % % % % % % 
% % % % % % % % % % % Xu = -(Cdu + 2*Cd)*qbar*S/(m*U1);
% % % % % % % % % % % Xa = -(Cda-Cl)*qbar*S/(m*U1);
% % % % % % % % % % % % % Xde = -(1/m) * Cdde*qbar*S;
% % % % % % % % % % % Xde = 0;
% % % % % % % % % % % Zu = (1/(m*U1)) * ((-Clu - 2*Cl)*qbar*S);
% % % % % % % % % % % Za = (1/m)*((-Cla-Cd)*qbar*S); % May have to do (1/(m*U1))
% % % % % % % % % % % Zde = -(1/m)*Clde*qbar*S;
% % % % % % % % % % % Zq = -qbar*S*cBar*Clq/(2*m*U1);
% % % % % % % % % % % Mu = (1/(Iy*U1))*(qbar*S*cBar*(CMu+2*CmM));
% % % % % % % % % % % Ma = qbar*S*cBar/Iy * Cma; % May have to divide by U1
% % % % % % % % % % % Madot = qbar*S*cBar^2*CmaDot/(2*Iy*U1);% May have to divide by U1
% % % % % % % % % % % Mq = qbar*S*cBar^2*Cmq/(2*Iy*U1);
% % % % % % % % % % % Mde = qbar*S*cBar*Cmde/Iy;


% % % This worked before
% % Along = [Xu Xa 0 -g/U1;
% %     Zu Za/U1 1 0;
% %     Mu+(Madot*Zu) Ma+(Madot*Za)/U1 Mq+Madot 0;
% %     0 0 1 0];
%}

g = 32.17405; % Gravity ft/s*s
qbar = 300;
S = 5500; % ft^2 % Wing area
cBar = 27.3; % cord width
W = 636636; % lb
m = W/g;
Iy = 3.31*10^6;
U1 = 502;
Xu = -.00499;
Cda = .17;
Cl = .4;
Xa = -(Cda-Cl)*qbar*S/(m*U1);
Zu = -.0807;
Cla = 4.3;
Cd = .021;
Za = (1/m)*((-Cla-Cd)*qbar*S);
Mu = .000146;
CmaDot = -19;
Madot = qbar*S*cBar^2*CmaDot/(2*Iy*U1);
Cma = -1;
Ma = qbar*S*cBar/Iy * Cma;
Mq = -.699;
Mde = -1.40;
Zde = -21.8;
Xde = 1.18;



Along = [Xu Xa 0 -g;
        Zu/U1 Za/U1 1 0;
        Mu+Madot*Zu/U1 Ma + Madot*Za/U1 Mq+Madot 0;
        0 0 1 0];


Blong = [Xde;Zde/U1;Mde+(Madot*Zde)/U1;0];
Clong=eye(length(Along));
Dlong = zeros(size(Clong,1),size(Blong,2));



% Open-loop longitudinal State-space system

states = {'udot [ft/s/s]' 'alphaDot [°/s]' 'qdot [°/s/s]' '\theta dot'};
inputs = {'Elevator'};
outputs = {'u [ft/s]' 'alpha' 'q [°/s]' '\theta'};

longSIMO = ss(Along,Blong,Clong,Dlong,'statename',states,...
'inputname',inputs,...
'outputname',outputs);


% Plot open-loop response
damp(longSIMO)
pzmap(longSIMO)

figure()
step(longSIMO),grid on, grid minor;
set(gca,'fontsize',14)

figure()
impulse(longSIMO),grid on, grid minor;
set(gca,'fontsize',14)


% Look at root locus
sys11=longSIMO('\theta','elevator');
figure()
bode(sys11)
figure()
rlocus(sys11)
sgrid;



deCommand = 1*5; % 5 degree elevator command
ton = 1; % Step command on
toff = 2; % Step command off
tfin = 400; % Sim time, [s]


% Design LQR Controller for Longitudinal Control

% First Check Controllability
ctrlLong = ctrb(longSIMO);

% Verify Full Rank
ctrlCheck = rank(ctrlLong);


Clong_alpha = [0 1 0 0]; % Output AoA
CBinv = inv(Clong_alpha*Blong);
CA = Clong_alpha*Along;
k = 5;

sys = 'dynInvtest';
out = sim(sys);




figure()
plot(out.tout,out.alpha_open,'k','linewidth',4),hold on;
plot(out.tout,out.disturbance,'b','linewidth',3),hold on, grid on, grid minor
plot(out.tout,out.aCommand,'r','linewidth',3),hold on, grid on, grid minor
xlabel('time [s]')
ylabel('a');
title('Open Loop Response to A.O.A Disturbance');
legend('OL','Disturbance','Command A.O.A');
set(gca,'fontsize',14)




figure()
plot(out.tout,out.alpha_dyn,'k','linewidth',4),hold on, grid on, grid minor;
plot(out.tout,out.disturbance,'b','linewidth',3),hold on, grid on, grid minor
plot(out.tout,out.aCommand,'r--','linewidth',3),hold on, grid on, grid minor
xlabel('time [s]')
ylabel('a');
title('Closed Loop Response to A.O.A Disturbance');
legend('CL','Disturbance','Command A.O.A');
set(gca,'fontsize',14)



figure()
plot(out.tout,out.de,'b','linewidth',3),hold on, grid on, grid minor
xlabel('time [s]')
ylabel('de [°]');
title('Elevator Deflction');
set(gca,'fontsize',14);



