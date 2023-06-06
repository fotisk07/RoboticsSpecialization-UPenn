
function u = controllerNoisyEnc(params, t, obs, th, dth)
  % This is the starter file for the week5 assignment
  % Now you only receive noisy measurements for theta, and must use your EKF from week 3 to filter the data and get an estimate of the state
  % obs = [ay; az; gx] (same as last week)
  % New for 6b: you also have access to params.traj(t)

  % Template code (same as last week)
  xhat = EKFupdate(params, t, obs);
  phi = xhat(1);
  phidot = xhat(2);

  % fill this out
%   FPhi=100;FTh = 50; ZPhi=0.9; ZTh=0.9;
%   KpPhi = (2*pi*FPhi)^2;
%   KdPhi = 4*pi*FPhi*ZPhi;
%
%   KpTh = (2*pi*FTh)^2;
%   KdTh = 4*pi*FTh*ZTh;
%   KpPhi=0.15; KdPhi=0.15; KpTh=0.1; KdTh=0.1;
%

  x = params.r * (th + phi);
  xdot = params.r * (dth + phidot);

  kp_x = 0.21; %% this are tuning values for inner controller (phi)
  kd_x = 0.5;

  e_x = params.traj(t) - x;
  edot_x = 0 - xdot;

  u_x = kp_x * e_x + kd_x * edot_x;
  phi_des = asin(u_x);

  kp_phi = 0.1; % these are tuning values for second outer controller. We use phi to control theta. This is a cascade control system.
  kd_phi = 0.01;

  e_phi = phi_des - phi;
  edot_phi = 0 - phidot;

  u_phi = kp_phi * sin(e_phi) + kd_phi * edot_phi;
  u = -u_phi;

end

function xhatOut = EKFupdate(params, t, z)

   % z = [ay; az; gx] with a* in units of g's, and gx in units of rad/s
  % You can borrow most of your week 3 solution, but this must only implement a single predict-update step of the EKF
  % Recall (from assignment 5b) that you can use persistent variables to create/update any additional state that you need.

  % Student completes this
  Q = diag([0.01 0.01]);
  R = diag([0.1 0.1 0.1]);
  xhat = [0;0];
  P = eye(2);

    persistent  time_ekf
    if isempty(time_ekf)
      %P = eye(2);
      time_ekf = 0;
      %xhat = [0;0];
    end


  dt = t - time_ekf;
  A = [1 dt; 0 1];
  xhat = A*xhat;
  P = A*P*A' + Q;

  phi = xhat(1);
  phidot = xhat(2);

  H = [cos(phi) -sin(phi) 0; 0 0 1]';
  K = P*H'/(H*P*H' + R);

  h = [sin(phi) cos(phi) phidot]';
  xhat = xhat + K*(z - h);
  P = (eye(2) - K*H)*P;

  time_ekf = t;
  xhatOut = xhat;

end
