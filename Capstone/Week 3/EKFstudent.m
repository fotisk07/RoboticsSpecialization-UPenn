

function xhat = EKFstudent(t, z)
  % In this exercise, you will batch-process this data: you are provided a 
  % vector of timestamps (of length T), and a 3xT matrix of observations, z.
  xhat = zeros(2,length(t));
  P = 1e3*eye(2);
  Q = diag([10000, 0.6]);
  R = diag([0.001, 0.01, 30]);
  for k=2:length(t)
      dt = t(k)-t(k-1);
      A = [1 dt; 0 1];
      xhat(:,k) = A*xhat(:, k-1);
      P = A*P*A'+Q;
      H = [cosd(xhat(1,k-1)) 0; -sind(xhat(1,k-1)) 0 ; 0 1]*0.3;
      h = [sind(xhat(1,k-1)); cosd(xhat(1,k-1)); xhat(2,k-1)];

      K = P*H'*inv(H*P*H'+R);
      xhat(:,k) = xhat(:,k) + K*(z(:,k) - h);
      P = (eye(2)-K*H)*P;
  end