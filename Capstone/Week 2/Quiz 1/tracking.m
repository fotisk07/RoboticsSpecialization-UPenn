

function tracking()
  % plots
  close all
  figure
  subplot(3,1,1)
  simAndPlot(@trajStep)

  subplot(3,1,2)
  simAndPlot(@trajRamp)

  subplot(3,1,3)
  simAndPlot(@trajSin)
  
 
end

function simAndPlot(trajToUse)
  params = struct();
  params.traj = trajToUse;
  tspan=[0,10];
  X0 = [0,0];
  [t, X] = ode45(@(t, X) doubleIntegrator(params, t, X), tspan, X0);
  hold all
  plot(t, params.traj(t), 'k--')
  plot(t, X(:,1))
  xlabel('t (sec)')
  ylabel('x (m)')
  legend({'Goal', 'Your controller'})
  hold off
end

function Xd = doubleIntegrator(params, t, X)
  u = controller(params, t, X(1), X(2));
  u = max(min(u, 5), -5);
  Xd = [X(2);u];
end

% TRAJECTORIES TO TRACK
function x = trajStep(t)
  if numel(t) == 1
    if t > 5
      x = -1;
    else
      x = 1;
    end
  else
    x = ones(size(t));
    for i=1:numel(t)
      if t(i) > 5
        x(i) = -1;
      end
    end
  end
end
function x = trajRamp(t)
  x = t;
end
function x = trajSin(t)
  x = sin(t);
end