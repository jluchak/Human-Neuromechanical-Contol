%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function was altered by:
% Jenna Luchak
% CID: 01429938
% For Human Neuromechanical Control: Tutorial #5
% March 8, 2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% z - State variable over time
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [z] = KalmanFilter_delay(A,C,Q,R,z0,P0,y)

    % Allocate memory for speed
    z = zeros(size(z0,1),length(y));
    % Initialize Kalman filter
    z(:,1) = z0;
    P = P0;

    % Kalman filter loop
    for i=11:1:length(y)-1
        % Prediction
        z_Prior = A*z(:,i-10);
        P = A*P*A'+Q;

        % Correction
        K = P*C'/(C*P*C'+R);
        z(:,i+1-10) = z_Prior + K*(y(:,i-10)-C*z_Prior);
        P = (eye(3)-K*C)*P;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% End of Function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%