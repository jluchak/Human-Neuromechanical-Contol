%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script was prepared by:
% Jenna Luchak
% CID: 01429938
% For Human Neuromechanical Control: Tutorial #5
% March 8, 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc; close all;

% Discretization of continuous system
dt = 0.01;

% Generate time array
t = 0:dt:4*pi;

%%%%%%% Kalman filter matrices %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% State transition matrix
A = [1 dt dt^2/2;
     0  1     dt;
     0  0     1];

% System noise and covariance matrix
SigmaQ = 0.3;
Q=[ SigmaQ^6/36    SigmaQ^5/12   SigmaQ^4/6
    SigmaQ^5/12    SigmaQ^4/4    SigmaQ^3/2
    SigmaQ^4/6     SigmaQ^3/2    SigmaQ^2];

% Observation matrix
C_1 = [1 0 0]; % Part A
C_2 = [1 0 0;1 0 0]; % PArt B and C

for i=1:2
% Observation noise
Sigma_A = [1.5 4.5]; % Question A
Sigma_pB = 1.5; Sigma_vB = 1.5; % Question B
Sigma_pC = 4.5; Sigma_vC = 1.5; % Question C

% Covariance matrix
R_A=(Sigma_A(i)^2); % Question A
R_B = [Sigma_vB^2 0;0 Sigma_pB^2]; % Question B
R_C = [Sigma_vC^2 0;0 Sigma_pC^2]; % Question C

% Initialize state and error covariance matrices
xInit = zeros(3,1);
PInit = diag([1 1 1]).*10^5; % Change to diag([1 1 1]); for part A

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Generate signal
Signal_A(i,:) = 40*sin(t); % Question A
Signal_BC =[40*sin(t);40*sin(t)]; % Question B and C

% Generate noise corrupted signal
SignalNoisy_A(i,:) = Signal_A(i,:) + sqrt(R_A)*randn(size(R_A,1),length(Signal_A(i,:))); % Part A or Part B or C with 1 sensor
SignalNoisy_B = Signal_BC + sqrt(R_B)*randn(size(R_B,1),length(Signal_BC)); % Part B with 2 sensors
SignalNoisy_C = Signal_BC + sqrt(R_C)*randn(size(R_C,1),length(Signal_BC)); % Part C with 2 sensors

% LQE function
S_A(i).a = KalmanFilter(A,C_1,Q,R_A,xInit,PInit,SignalNoisy_A(i,:)); % Part A or Part B or C with 1 sensor
S_B = KalmanFilter(A,C_2,Q,R_B,xInit,PInit,SignalNoisy_B(i,:)); % Part B with 2 sensors
S_C = KalmanFilter(A,C_2,Q,R_C,xInit,PInit,SignalNoisy_C); % Part C with 2 sensors
S_D = KalmanFilter_delay(A,C_1,Q,R_A,xInit,PInit,SignalNoisy_A(1,:)); % Part D with sigma = 1.5

end
% Re define Kalman filtering outputs from struct to variable
x_15 = S_A(1).a; % Part A
x_45 = S_A(2).a; % Part A

% Square Estimation Error for Part B and C only
error_1 = norm(x_15(1,:)-Signal_A(1,:))^2/length(t); % One sensor
error_2B = norm(S_B(1,:)-Signal_BC(1,:))^2/length(t); % Two sensors
error_2C = norm(S_C(1,:)-Signal_BC(1,:))^2/length(t); % Two sensors
error_D = norm(S_D(1,:)-Signal_A(1,:))^2/length(t); % Two sensors

%%  Plots Part A
set(0,'DefaultFigureWindowStyle','docked')
figure(1);set(gcf,'color','white');
set(gca,'FontSize',14)
hold on;
% When sigma = 1.5
plot(t,Signal_A(1,:),'--k','linewidth',2.5); % True signal
plot(t,SignalNoisy_A(1,:),'r'); % noisy signal 1.5
plot(t,x_15(1,:),'b','linewidth',1.5); %filter 1.5
% When sigma = 4.5
plot(t,SignalNoisy_A(2,:),'m'); % noisy 4.5
plot(t,x_45(1,:),'g','linewidth',1.5); % filter 4.5
hold off;
legend('True position','Noisy position \sigma = 1.5cm','Filtered Signal \sigma = 1.5cm',...
   'Noisy position \sigma = 4.5cm','Filtered Signal \sigma = 4.5cm');
xlabel('Time (s)','fontsize',15);
ylabel('Position (cm)','fontsize',15);

%% Part B
%One Sensor
figure(2);set(gcf,'color','white');
subplot(2,1,1)
set(gca,'FontSize',16)
hold on;
plot(t,Signal_BC(1,:),'--k','linewidth',2.5); % True signal
plot(t,x_15(1,:),'b','linewidth',1.5); % filter 
hold off;
legend('True position','Filtered Signal');
xlabel('Time (s)','fontsize',15);
ylabel('Position (cm)','fontsize',15);
title('One Sensor - Vision Only')
hold off
% Two Sensors
subplot(2,1,2)
set(gca,'FontSize',16)
hold on;
plot(t,Signal_BC(1,:),'--k','linewidth',2.5); % True signal
plot(t,S_B(1,:),'b','linewidth',1.5); % filter
hold off;
legend('True position','Filtered Signal');
xlabel('Time (s)','fontsize',15);
ylabel('Position (cm)','fontsize',15);
title('Two sensors - Vision and Proprioception');

%% Part C - 
figure(3);set(gcf,'color','white');
set(gca,'FontSize',16)
hold on;
plot(t,Signal_BC(1,:),'--k','linewidth',2.5); % True signal
% One Sensor
plot(t,x_15(1,:),'b','linewidth',1.5); % filter 
% Two Sensors
plot(t,S_C(1,:),'g','linewidth',1.5); % filter
hold off;
legend('True position','Filtered Signal - One Sensor:Vision Only',...
    'Filtered Signal - Two Sensors:Vision and Propriception','location','northoutside');
xlabel('Time (s)','fontsize',15);
ylabel('Position (cm)','fontsize',15);

%% Part D
figure(4);set(gcf,'color','white');
set(gca,'FontSize',14)
hold on;
plot(t,Signal_A(1,:),'--k','linewidth',2.5); % True signal
plot(t,SignalNoisy_A(1,:),'r'); % noisy signal
plot(t,x_15(1,:),'b','linewidth',1.5); %filtered
plot(t,S_D(1,:),'g','linewidth',1.5); % Delayed
hold off;
legend('True position','Noisy position','Filtered Signal',...
   'Sensory Delay');
xlabel('Time (s)','fontsize',15);
ylabel('Position (cm)','fontsize',15);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




