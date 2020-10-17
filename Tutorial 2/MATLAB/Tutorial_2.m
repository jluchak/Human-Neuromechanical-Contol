%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script was prepared by:
% Jenna Luchak
% CID: 01429938
% For Human Neuromechanical Control: Tutorial #2
% February 8, 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clear Workspace and Command Window

clc;
clear;

%% Define the time constraints of the project

ts = 0.01; % Time step in seconds
T = 2; % Total time duration in seconds
t = 0:ts:T; % Time Vector

%% Define the joint angles and joint velocity matrices
% i.e. Initialize two empty matrices with 3 rows.
% First row represents the shoulder joint, second row the elbow joint and 
% third row is the wrist joint values.

q = zeros(3,T/ts+1); 
q(:,1) = [deg2rad(90) deg2rad(130) deg2rad(90)]; % Initial angles
q_dot = zeros(3,T/ts+1);

%% Define the initial legths of the arm system

l_s = 0.3; % Upper arm length in meters
l_e = 0.3; % Forearm length in meters
l_w = 0.15; % Hand length in meters

%% Given Equations for desired position and velocity of the arm endpoint

g = (6*t.^5/(T.^5))-(15*t.^4/(T.^4))+(10*t.^3/(T.^3)); % Intermediary Equation
g_dot = (30*t.^4/(T.^5))-(60*t.^3/(T.^4))+(30*t.^2/(T.^3)); % Derivative of intermediary equation

x = -0.1334+0.1289*g; % Desired position in x direction [m]
x_dot = 0.1289*g_dot; % Desired velocity in x direction [m/s]

y = -0.0077+0.7355*g; % Desired position in y direction [m]
y_dot = 0.7355*g_dot; % Desired velocity in y direction [m/s]

% Combine the x and y component velocities into one matrix
velocity = [x_dot;y_dot];

%% Calculate the joint angle profile, speed profile and position
% i.e. use a "for" loop to determine the actual joint angles and 
% angular velocities at every time step up to the specified time.

for i=2:1:(T/ts+1)
    
% Update joint angle matrix    
    q(:,i)=q(:,i-1)+q_dot(:,i-1)*ts;
    
% Calculate the Jacobian
    J=[(-l_s*sin(q(1,i))-l_e*sin(sum(q(1:2,i)))-l_w*sin(sum(q(:,i)))) (-l_e*sin(sum(q(1:2,i)))-l_w*sin(sum(q(:,i)))) (-l_w*sin(sum(q(:,i)))); (l_s*cos(q(1,i))+l_e*cos(sum(q(1:2,i)))+l_w*cos(sum(q(:,i)))) (l_e*cos(sum(q(1:2,i)))+l_w*cos(sum(q(:,i)))) (l_w*cos(sum(q(:,i))))];

% Calculate the Psuedo Inverse of the Jacobian
    pseudo = pinv(J);
    
% Compute the angular velocity of the arm    
    q_dot(:,i)=pseudo*velocity(:,i);

end

%% Print a plot of the end-effector trajectory
figure(1)
plot(x,y);
grid on
xlabel('X-position [m]'); % x axis title
ylabel('Y-position [m]'); % y axis title
title('End-point Trajectory in Task Space');
set(gca,'FontSize',14);

%% Print a plot of the end-point position with respect to time
figure(2)
grid on
hold on
plot(t,x,'c'); % x component of end effector position
plot(t,y,'r'); % y component of end effector position
legend('X Component','Y Component','location','best');
xlabel('Time [seconds]'); % x axis title
ylabel('Position [meters]'); % y axis title
title('End-point Position with Respect to Time');
set(gca,'FontSize',14)
hold off

%% Print a plot of the end-point velocity with respect to time
figure(3)
grid on
hold on
plot(t,x_dot,'m'); % x component of end effector speed
plot(t,y_dot,'b'); % y component of end effector speed
legend('X Component','Y Component','location','best');
xlabel('Time [seconds]'); % x axis title
ylabel('Speed [m/s]'); % y axis title
title('End-point Velocity with Respect to Time');
set(gca,'FontSize',14)
hold off

%% Print a plot of the joint angles with respect to time
figure(4)
grid on
hold on
plot(t,q(1,:),'b'); % Shoulder joint
plot(t,q(2,:),'r'); % Elbow joint
plot(t,q(3,:),'g'); % Wrist joint
legend('q_s','q_e','q_w','location','best');
xlabel('Time [seconds]'); % x axis title
ylabel('Joint Angles [radians]'); % y axis title
title('Joint Anglular Position with Respect to Time');
set(gca,'FontSize',14)
hold off

%% Print a plot of the angular joint velocity with respect to time
figure(5)
hold on
grid on
plot(t,q_dot(1,:),'b'); % Shoulder joint velocity
plot(t,q_dot(2,:),'r'); % Elbow joint velocity
plot(t,q_dot(3,:),'g'); % Wrist joint velocity
legend('w_s','w_e','w_w','location','best');
xlabel('Time [seconds]'); % x axis title
ylabel('Joint Angular Velocity [rads/s]'); % y axis title
title('Joint Angular Velocity with Respect to Time');
set(gca,'FontSize',14)
hold off

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%