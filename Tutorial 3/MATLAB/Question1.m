%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script was prepared by:
% Jenna Luchak
% CID: 01429938
% For Human Neuromechanical Control: Tutorial #3 - Question 1
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
T = 20;     % In seconds

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
q_start = [ 0*pi/180, 30*pi/180]; 
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
  Kd=30;
  
for part = 1:2  
for i=1:T_samples
    
   % WRITE HERE THE CONTROLLER EQUATION
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
   % Question A
   if part ==1
   if i > 200
           Torque = [0 -0.1*qdot(i,2)]';
   else
           Torque = [0 0.02-0.1*qdot(i,2)]';
   end 
   else
   % Question 1B
   if i > 200
           Torque = [-0.1*qdot(i,1) -0.1*qdot(i,2)]';
   else
           Torque = [-0.1*qdot(i,1) 0.02-0.1*qdot(i,2)]';
   end
   end
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
Q(part).a = q; 
% q=[];
% qdot = [];
end

q_A = Q(1).a;
q_B = Q(2).a;

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

% Question 1.a
figure,
hold on
grid on
plot(t,q_A(:,2)*180/pi,'b','linewidth',2);
%plot(t,q_A(:,1)*180/pi,'b','linewidth',2);
xlabel('Time [s]');
ylabel('Angle [degree]');
legend('Elbow, q_e','Location','Best');
set(gca,'fontsize',20);

% Question 1.b
figure,
hold on
grid on
plot(t,q_B(:,1)*180/pi,'r','linewidth',2);
plot(t,q_B(:,2)*180/pi,'b','linewidth',2);
plot(t,q_A(:,2)*180/pi,'g--','linewidth',2);
xlabel('Time [s]');
ylabel('Angle [degree]');
legend('Shoulder, q_s','Elbow, q_e','Elbow with Fixed Shoulder, q*_e','Location','Best');
set(gca,'fontsize',20);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Script for Question 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

