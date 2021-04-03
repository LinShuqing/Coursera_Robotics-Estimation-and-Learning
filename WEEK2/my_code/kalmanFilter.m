function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y
%   state 是1*4向量，t 为下标， 模型为二维运动模型
%   DM:X(t+1)=A*X(t)+w(t)
%   MM:Z(t)=C*X(t)
%   K=P*C'*(R+C*P*C')^(-1)
%   P=A*P(t-1)*A'+S_m
%   R=C*P(t)*C'+S_o
    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)
    % dt = 0.033;
    % 噪声一开始比较小
    Sigma_m = 0.001 * eye(4); % 运动模型误差
    Sigma_o = 0.001 * eye(2); % 观察模型误差
    % 转移矩阵
    % state = [position_x, position_y, velocity_x, velocity_y] %1 by 4
    % position_x=position_x+velocity_x*dt
    % position_y=position_y+velocity_y*dt
    % 速度不变

    
    % 观测矩阵
    % zx=position_x
    % zy=position_y
    C = [1 0 0 0;
         0 1 0 0];

    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.001 * eye(4);
        param.A = [[1,0,0,0]',[0,1,0,0]',[0.033,0,1,0]',[0,0.033,0,1]'];
        predictx = x;
        predicty = y;
        return;
    end
    
    
%{    
    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    vx = (x - state(1)) / (t - previous_t);
    vy = (y - state(2)) / (t - previous_t);
    % Predict 330ms into the future
    predictx = x + vx * 0.330;
    predicty = y + vy * 0.330;
    % State is a four dimensional element
    state = [x, y, vx, vy];
 %}
    % My Kalman filter updates
    dt = t - previous_t;
    param.A = [[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];
    Xkp=param.A*state'; 
    % 更新P,R
    Pkp=param.A*param.P*param.A'+Sigma_m;
    % Rkp=Sigma_o;
    Rkp=C*Pkp*C'+Sigma_o;
    % 计算卡尔曼增益
    K=Pkp*C'/(Rkp+C*Pkp*C');
    Z=[x,y]';
    % 状态更新和状态协方差更新
    param.P = Pkp-K*C*Pkp;
    state = (Xkp + K*(Z-C*Xkp))';
    % 预测
    % position_x=position_x+velocity_x*dt
    % position_y=position_y+velocity_y*dt
    predictx = state(1);
    predicty = state(2);
    
    
end
