
function u = controllerNoisy(params, t, obs)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for phi, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] with a* in units of g's, and gx in units of rad/s

  % This template code calls the function EKFupdate that you must complete below
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % The rest of this function should ideally be identical to your solution in week 4
  % Student completes this
      persistent newstate
  
  if isempty(newstate)
    % initialize
    newstate = 0;
  end
  persistent time
  if isempty(time)
    % initialize
    time = t;
  end
  
  dt = t - time;
  time = t;
  
  kp=32;
  kd=2;
  ki=1100;
  newstate=newstate + (0-phi*dt);
  
  u=kp*(0-phi) + kd*(0-phidot) + ki*(newstate);
  u=-u;
  
 
end

function xhatOut = EKFupdate(params, t, z)
  % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

  
  persistent P xhat time2 k
  if isempty(P)
    P=1e3*eye(2);
    xhat = zeros(2,length(t));
    time2 = t;
    k = 1;
  end
  
  k = k+1;
  Q = diag([10000, 0.7]);
  R = diag([0.001, 0.05, 30]);
  dt = t - time2;
  time2 = t;
  A = [1 dt; 0 1];
  xhat(:,k) = A*xhat(:, k-1);
  P = A*P*A' + Q;
  H = [cos(xhat(1,k-1)) 0; -sin(xhat(1,k-1)) 0 ; 0 1];
  h = [sin(xhat(1,k-1)); cos(xhat(1,k-1)); xhat(2,k-1)];
  %       h = h + H*(xkp - xhat(:,k-1));
  K = P*H'*inv(H*P*H'+R);
  xhat(:,k) = xhat(:,k) + K*(z - h);
  P = (eye(2)-K*H)*P;
  xhatOut = xhat(:, k);
end
  
  
 

