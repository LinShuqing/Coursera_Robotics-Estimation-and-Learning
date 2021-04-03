function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    dt = 0.033;
      % Noise covariance matrices
    Sigma_m = 0.01 * eye(4);
    Sigma_o = 0.01 * eye(2);
    
      % Transition matrix
    A = [1 0 dt 0;
        0 1 0 dt;
        0 0 1 0;
        0 0 0 1];
    
    % Observation matrix
    C = [1 0 0 0;
         0 1 0 0];
    
    % Check if the first time running this function
    if previous_t<0           
        state = [x, y, 0, 0];
        param.P = 0.001 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
      
    % Predicted state & covariance matrix
    Xkp = A*state';
    Pkp = A*param.P*A'+Sigma_m;
    R=Sigma_o;
    % Kalman gain
    K = Pkp*C'/(C*Pkp*C'+R);
        
    % Y/Z measrement
    Y = [x y]';
    
    % Update covariance matrix  
    param.P = Pkp-K*C*Pkp;
    
    % Update state
    state = (Xkp+K*(Y-C*Xkp))';
    
    % Prediction
    predictx = state(1)+dt * state(3);
    predicty = state(2)+dt * state(4);
    
end
