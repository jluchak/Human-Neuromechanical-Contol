%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script was prepared by:
% Jenna Luchak
% CID: 01429938
% For Human Neuromechanical Control: Tutorial #3 - Question 2
% February 18, 2018
%
% With reference to
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simone Tanzarella 04/02/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear Workspace and Command Window

clc;
clear;

%% Initialisation

% Define sampling rate and trajectory duration
dt = 0.01;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Change the sampling time
% Question 1, T = 20s
% Quesiton 2, T = 1.0s
T = 1.0; 
% T = 1.0;
% T = 0.2;
T_samples = fix(T/dt);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Kinematic parameters
l = [0.31,0.34]; % m
% Dynamic parameters
m = [1.93,2.04]; % kg
I = [0.0141,0.0188]; %Inertia moments kg*m^2
cL = [0.165,0.2]; % m

% Start posture
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Question 1a [qs;qe] = [0 30]
% Question 1b [qs;qe] = [0 30]
% Question 2 [qs;qe] = [90 130]
q_start = [ 90*pi/180, 130*pi/180]; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Compute reference trajectory
[qr,qr_dot,xp] = planned_trajectory(l,dt,T,q_start);

%% Initialisation

q = zeros(T_samples,2);
q(1,:) = q_start;
qdot = zeros(T_samples,2);
qdot(1,:) = [0,0];

% Integration functions

UpdateAngle = @(q,qdot)([q(1)+dt*qdot(1);
                       q(2)+dt*qdot(2)]); 

UpdateVel = @(qdot,qddot)([qdot(1)+dt*qddot(1);
                       qdot(2)+dt*qddot(2)]); 
                   
JointAccel = @(Torque,H,Cqdot)(H\(Torque-Cqdot)); 

% PD constants respectively proportional and derivative
  Kp=100;
  Kd=10;
  
for i=1:T_samples
    
   % WRITE HERE THE CONTROLLER EQUATION
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

    Torque = Kp*(qr(i,:)-q(i,:))+Kd*(qr(i,:)-q(i,:));
       
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
    if size(Torque,2) > size(Torque,1)
        Torque = Torque';
    end
    
    % Compute dynamics
    H= mass(m,l,cL,I,q(i,:));
    Cqdot = coriolis(m,l,cL,q(i,:),qdot(i,:));
    qddot = JointAccel(Torque,H,Cqdot); 

    % Movement integration
    qdot(i+1,:) = UpdateVel(qdot(i,:),qddot);
    q(i+1,:) = UpdateAngle(q(i,:),qdot(i+1,:));
    
end

for i = 1:size(q,1)

    X= kin(l,q(i,:));
    x(i) = X(1);
    y(i) = X(2);

    X= kin(l,qr(i,:));
    xr(i) = X(1);
    yr(i) = X(2);

end

%% Plots

t = 0:dt:T;

% Question 2.a
figure,
grid on
plot(t,q(:,1)*180/pi,'r','linewidth',2);
hold on
plot(t,qr(:,1)*180/pi,'m--','linewidth',2);
hold on
plot(t,q(:,2)*180/pi,'b','linewidth',2);
hold on
plot(t,qr(:,2)*180/pi,'c--','linewidth',2);
xlabel('time [s]');
ylabel('angle [degree]');
legend('Actual Shoulder, q_s','Planned Shoulder, q*_s ','Actual Elbow, q_e','Planned Elbow, q*_e','location','Best');
set(gca,'fontsize',18);

% Question 2.b
figure,
grid on
plot(t,x,'r','linewidth',2);
hold on
plot(t,xr,'m--','linewidth',2);
hold on
plot(t,y,'b','linewidth',2);
hold on
plot(t,yr,'c--','linewidth',2);
xlabel('time [s]');
ylabel('position [m]');
legend('Actual, x','Desired, x*','Actual, y','Desired, y*','Location','Best');
% legend('elbow')
set(gca,'fontsize',18);

figure, 
grid on
plot(x,y,'k','linewidth',2); 
hold on, 
plot(xr,yr,'k--','linewidth',2);
axis equal
legend('Actual trajectory', 'Reference trajectory','location','Best');
xlabel('x position [m]');
ylabel('y position [m]');
set(gca,'fontsize',18);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Script for Question 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

