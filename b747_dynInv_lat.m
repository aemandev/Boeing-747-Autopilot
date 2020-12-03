close all; clear all; clc
%%%%%%%%%%%%%%%%%%%%%%%
% Flgiht # 3
g = 32.174;
theta1 = deg2rad(6.8);
U1 = 502;
b = 195.7;
W = 636636;
Ix = 18.2*10^6;
Iz = 49.7*10^6;
Ixz = 970056;

iz = 0;
ix = 0;

Yv = -0.143;
Yp = 0;
Yr = 0;
Yb = -27.8;
Lb = -3.19;
Lp = -1.12;
Lr = 0.379;
Nb = .810;
Np = -.0706;
Nr = -.246;
Ydr = .0226;
Ldr = .254;
Lda = .229;
Ndr = -.614;
Nda = .0285;
Yda = 0;



Alat = [0 1 0 0 0;
    0 Lp Lb Lr 0;
    g*cos(theta1)/U1 Yp/U1 Yb/U1 Yr/U1 - 1 0;
    0 Np Nb Nr 0;
    0 0 0 1 0];


Blat = [0 0 ;
    Ldr Lda;
    Ydr/U1 Yda/U1;
    Ndr Nda;
    0 0];
Clat=eye(5);
Dlat = zeros(size(Clat,1),size(Blat,2));

states = {'phiDot' 'Roll Rate' 'Side slip Rate' 'yaw rate rate' 'Yaw Rate'};
inputs = {'rudder' 'aileron'};
outputs = {'Roll Angle' 'Roll Rate' 'Side slip Angle' 'yaw rate' 'Yaw Angle'};
sys_mimo = ss(Alat,Blat,Clat,Dlat,'statename',states,...
'inputname',inputs,...
'outputname',outputs);



%tf(sys_mimo)
t = 0:.01:100;

uZeros = zeros(length(t),1);
u = zeros(length(t),1);
u(t>=1) = 1;
% Rudder and Aileron Step
RudderStep = [u uZeros];
AileronStep = [uZeros u];
u(t>=2) = 0;

% Rudder and Aileron Impulse
RudderImpulse = [u uZeros];
AileronImpulse = [uZeros u];


% Step Response
[yRudderStep,tRudderStep,xRudderStep] = lsim(sys_mimo,RudderStep,t);
[yAileronStep,tAileronStep,xAileronStep] = lsim(sys_mimo,AileronStep,t);

% Impulse Response
[yRudderImpulse,tRudderImpulse,xRudderImpulse] = lsim(sys_mimo,RudderImpulse,t);
[yAileronImpulse,tAileronImpulse,xAileronImpulse] = lsim(sys_mimo,AileronImpulse,t);
damp(sys_mimo);

% Open-loop longitudinal State-space system


% Plot Impulse Response

figure()
plot(t,RudderImpulse(:,1),'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('deg [°]')
xlim([0 10])
title('Impulse Response - Rudder Deflection')
set(gca,'fontsize',13);



figure()

subplot(5,1,1)
plot(tRudderImpulse,yRudderImpulse(:,1),'b-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('Bank Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,2)
plot(tRudderImpulse,yRudderImpulse(:,2)*180/pi,'m-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Roll Rate [°/s]')
set(gca,'fontsize',13);


subplot(5,1,3)
plot(tRudderImpulse,yRudderImpulse(:,3)*180/pi,'g-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Side Slip Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,4)
plot(tRudderImpulse,yRudderImpulse(:,4)*180/pi,'r-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Rate [°/s]')
set(gca,'fontsize',13);



subplot(5,1,5)
plot(tRudderImpulse,yRudderImpulse(:,5)*180/pi,'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Angle [°]')
sgtitle('Lateral Impulse Response of 747 SuperTanker from Unit Rudder Deflection')
set(gca,'fontsize',13);




figure()
plot(t,AileronImpulse(:,2),'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('deg [°]')
xlim([0 10])
title('Impulse Response - Aileron Deflection')
set(gca,'fontsize',12);



figure()

subplot(5,1,1)
plot(tAileronImpulse,yAileronImpulse(:,1),'b-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('Bank Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,2)
plot(tAileronImpulse,yAileronImpulse(:,2)*180/pi,'m-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Roll Rate [°/s]')
set(gca,'fontsize',13);


subplot(5,1,3)
plot(tAileronImpulse,yAileronImpulse(:,3)*180/pi,'g-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Side Slip Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,4)
plot(tAileronImpulse,yAileronImpulse(:,4)*180/pi,'r-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Rate [°/s]')
set(gca,'fontsize',13);



subplot(5,1,5)
plot(tAileronImpulse,yAileronImpulse(:,5)*180/pi,'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Angle [°]')
sgtitle('Lateral Impulse Response of 747 SuperTanker from Unit Aileron Deflection')
set(gca,'fontsize',13);



% Plot Step Response
figure()
plot(t,RudderStep(:,1),'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('deg [°]')
xlim([0 10])
title('Step Response - Rudder Deflection')
set(gca,'fontsize',13);



figure()

subplot(5,1,1)
plot(tRudderStep,yRudderStep(:,1),'b-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('Bank Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,2)
plot(tRudderStep,yRudderStep(:,2)*180/pi,'m-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Roll Rate [°/s]')
set(gca,'fontsize',13);


subplot(5,1,3)
plot(tRudderStep,yRudderStep(:,3)*180/pi,'g-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Side Slip Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,4)
plot(tRudderStep,yRudderStep(:,4)*180/pi,'r-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Rate [°/s]')
set(gca,'fontsize',13);



subplot(5,1,5)
plot(tRudderStep,yRudderStep(:,5)*180/pi,'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Angle [°]')
sgtitle('Lateral Step Response of 747 SuperTanker from Unit Rudder Deflection')

set(gca,'fontsize',13);



figure()
plot(t,AileronStep(:,2),'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('deg [°]')
xlim([0 10])
title('Step Response - Aileron Deflection')
set(gca,'fontsize',13);




figure()

subplot(5,1,1)
plot(tAileronStep,yAileronStep(:,1),'b-','linewidth',2), grid on, grid minor
xlabel('time [s]')
ylabel('Bank Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,2)
plot(tAileronStep,yAileronStep(:,2)*180/pi,'m-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Roll Rate [°/s]')
set(gca,'fontsize',13);


subplot(5,1,3)
plot(tAileronStep,yAileronStep(:,3)*180/pi,'g-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Side Slip Angle [°]')
set(gca,'fontsize',13);


subplot(5,1,4)
plot(tAileronStep,yAileronStep(:,4)*180/pi,'r-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Rate [°/s]')
set(gca,'fontsize',13);



subplot(5,1,5)
plot(tAileronStep,yAileronStep(:,5)*180/pi,'k-','linewidth',2), grid on, grid minor
xlabel('time [s]')
xlim([0 100])
ylabel('Yaw Angle [°]')
sgtitle('Lateral Step Response of 747 SuperTanker from Unit Aileron Deflection')
set(gca,'fontsize',13);

ctrlLat = ctrb(sys_mimo);

% Verify Full Rank
ctrlCheck = rank(ctrlLat);

% Start with roll angle autopilot hold
sys11 = sys_mimo('Roll Angle','aileron');
figure()
bode(sys11)
figure()
rlocus(sys11)
sgrid;
s = tf('s');
daTodPhiNum = Lda;
daToPhiDenom = s*(s-Lp);
rollApproxSys = daTodPhiNum/daToPhiDenom;
[num,den] = tfdata(rollApproxSys,'v');

phi_commandDeg = 5; % 5 deg roll command 
phi_command = deg2rad(phi_commandDeg);

actTF = 10/(s+10);
[numAct,denAct] = tfdata(actTF,'v');


sys = 'Learjet_HW2_problem2';
open_system(sys);
sysOut = sim(sys);



% Investiage Optimal Control LQR


Blat = Blat(:,2);
Clat=eye(5);
Dlat = zeros(size(Clat,1),size(Blat,2));
R = 1.2;
Q = diag([100 1 1 1 1]);
kLQR = lqr(Alat,Blat,Q,R);

% Gain Tuning
kp = 5;
ki = 1.9;
kd = 1.5;

sysOut = sim('lqrRoll');



figure()
plot(sysOut.tout, sysOut.disturbance,'b-','linewidth',2),hold on,grid on, grid minor
xlabel('time[s]');
ylabel('Disturbanace [°]');
title('Disturbance to Aircraft');

figure()
plot(sysOut.tout, sysOut.daCommand,'r--','linewidth',2),hold on
% plot(sysOut.tout, sysOut.disturbance,'b-','linewidth',2),hold on
plot(sysOut.tout,sysOut.RollOpenLoop,'k-','linewidth',2),hold on, grid on, grid minor

xlabel('t(sec)');
ylabel('Phi [deg]');
title('Open Loop Roll Angle Response to Disturbance');
legend('Command Roll','Roll Angle','location','best');
set(gca,'fontsize',14)



figure()
subplot(2,1,1)
plot(sysOut.tout, sysOut.daCommand,'r--','linewidth',2),hold on
% plot(sysOut.tout, sysOut.disturbance,'b-','linewidth',2),hold on
plot(sysOut.tout,sysOut.rollClosedLoop,'k-','linewidth',2),hold on, grid on, grid minor

xlabel('t(sec)');
ylabel('Phi [deg]');
title('Closed Loop Roll Angle Response to Disturbance');
legend('Command Roll','Roll Angle','location','best');
set(gca,'fontsize',14)


subplot(2,1,2);
plot(sysOut.tout, sysOut.da,'r-','linewidth',2),hold on, grid on, grid minor

xlabel('t(sec)');
ylabel('Aileron Deflection [deg]');
title(['Aileron Deflection Angle - Kp = ',num2str(kp), ' Kd = ',num2str(kd), ' Ki = ',num2str(ki)]);
set(gca,'fontsize',14)

