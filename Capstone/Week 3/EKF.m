
function EKF()
  % starter code for loading both trials and plotting the result
  load data.mat

  figure('Name', 'Trial 0')
  EKFfromData(trial0);
  figure('Name', 'Trial 1')
  EKFfromData(trial1);
end

function est = EKFfromData(a)
  ACCEL_SENS = 16384;
  GYRO_SENS = 131;
  ayz = a(:,2:3) ./ ACCEL_SENS;
  gx = a(:,4) ./ GYRO_SENS;
  t = a(:,1) ./ 1000;
  z = [ayz, gx]';

  xhat = EKFstudent(t, z);

  % PLOT
  clf;
  subplot(4,1,1)
  hold all
  plot(t, xhat(1,:))
  ylabel('$\phi$ (filter)','Interpreter','latex')
  hold off

  subplot(4,1,2)
  hold all
  plot(t, xhat(2,:))
  plot(t, gx)
  ylabel('$\dot\phi$ (filter)','Interpreter','latex')
  hold off

  accelPred = zeros(2,length(t));
  for i=1:length(t)
    accelPred(:,i) = [sind(xhat(1,i)),cosd(xhat(1,i))];
  end

  subplot(4,1,3)
  hold all
  plot(t, accelPred(1,:))
  plot(t, ayz(:,1))
  ylabel('$a_y$','Interpreter','latex')
  legend({'Filter', 'Measurement'})
  hold off

  subplot(4,1,4)
  hold all
  plot(t, accelPred(2,:))
  plot(t, ayz(:,2))
  ylabel('$a_z$','Interpreter','latex')
  legend({'Filter', 'Measurement'})
  xlabel('$t$ (sec)','Interpreter','latex')
  hold off

  est = xhat(1,:);
end

