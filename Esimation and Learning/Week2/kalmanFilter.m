function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    dt=t-previous_t;
    R = eye(2);
    C=[1 0 0 0;0 1 0 0];
    A=[1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1];
    Af=[1 0 0.33 0;0 1 0 0.33;0 0 1 0;0 0 0 1];
    Sm=diag([0.02 0.001 0.09 0.01]);
    R=diag([0.002 0.002]);
    
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4);
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    P=param.P;
    P=A*P*A'+Sm;
    
    K=P*C'*inv(R+C*P*C');
    xt=state';
    z=[x y]';
    x_hat=A*xt+K*(z-C*A*xt);
    x_f=Af*xt+K*(z-C*Af*xt);
%     x_f=Af*x_hat;
    state=x_hat';
    predictx=x_f(1);
    predicty=x_f(2);
    P=P-K*C*P;
    param.P=P;
    
    
    
    
    
    
    
    
%     vx = (x - state(1)) / (t - previous_t);
%     vy = (y - state(2)) / (t - previous_t);
%     % Predict 330ms into the future
%     predictx = x + vx * 0.330;
%     predicty = y + vy * 0.330;
%     % State is a four dimensional element
%     state = [x, y, vx, vy];



end
