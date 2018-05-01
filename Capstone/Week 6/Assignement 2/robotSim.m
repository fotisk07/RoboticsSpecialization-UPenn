
function robotSim()
  % This file is starter code
  params = struct();

  params.g = 9.81;
  params.mr = 0.25;
  params.ir = 0.0001;
  params.d = 0.1;
  params.r = 0.02;
  params.phiNoise = 0.01;
  params.dphiNoise = 0.01;

  % NOTE: change this between @trajStep and @trajSin to test
  params.traj = @trajStep;

  tsim = 10;
  y0=[0,0,0,0]';
  timestep = 0.01;

  % warning('off','MATLAB:illConditionedMatrix')

  dispstat('','init')
  for l=0:timestep:tsim-timestep
    % if you want more feedback as the sim is integrating, uncomment the following line (will count up to 10)
    dispstat(sprintf('t = %.2fs (out of %d)',l+timestep,tsim)); 
    [tl,Xl] = ode45(@(t,y) dynamics(params,t,y),[l,l+timestep-1e-6],y0);
    y0 = Xl(end,:);
    % add noise to the angles
    y0(2) = y0(2) + params.phiNoise*randn();
    y0(4) = y0(4) + params.dphiNoise*randn();
    % append the solution vectors
    if l==0
      X=Xl;
      fullt = tl;
    else
      X=[X;Xl];
      fullt=[fullt;tl];
    end
  end
  
  errVec(1) = sqrt(sum(((X(:,1)+X(:,2))*params.r - params.traj(fullt)).^2));
  errVec(2) = sqrt(sum(X(:,2).^2))
  % warning('on','MATLAB:illConditionedMatrix')

  close('all')
  figure(1)
  subplot(3,1,1)
  plot(fullt, X(:,2))
  ylabel('$\phi$ (rad)','Interpreter','latex')
  subplot(3,1,2)
  plot(fullt, X(:,4))
  ylabel('$\dot\phi$ (rad/s)','Interpreter','latex')
  subplot(3,1,3)
  hold all
  plot(fullt, params.traj(fullt), 'k--')
  plot(fullt, (X(:,1) + X(:,2)) * params.r)
  hold off
  ylabel('$x$ (m)','Interpreter','latex')
  xlabel('t (sec)')

  % animate
  framedel = 0.05;
  slowmo = 0;
  tdraw = 0:framedel:fullt(end);
  Xdraw = interp1(fullt, X, tdraw);
  % size(Xdraw)

  saveanim = 0;

  if saveanim
    v = VideoWriter('anim.avi','Uncompressed AVI');
    open(v);
  end
  
  for i=1:numel(tdraw)
    figure(2)
    clf
    hold all
    th = Xdraw(i,1);
    phi = Xdraw(i,2);
    dth = Xdraw(i,3);
    dphi = Xdraw(i,4);
    x = th*params.r;
    % body
    % corners = [-0.025,0.025;0,0.15];
    % cornersr = [cos(phi),-sin(phi);sin(phi),cos(phi)]*corners;
    % Rect2(x+cornersr(1,1),cornersr(2,1),x+cornersr(1,2),cornersr(2,2),0.1,0.1)
    line([x,x+2*params.d*sin(phi)],[0,2*params.d*cos(phi)],'Color',[.5,.5,.5],'Linewidth',12);
    % wheel
    circle(x, 0, params.r)
    % 
    line([-2,2],[-params.r,-params.r],'Color',[0.3,0.3,0.3])
    % 
    text(-0.1,0.4,['t = ',num2str(tdraw(i))])
    %
    line([params.traj(tdraw(i)),params.traj(tdraw(i))],[-0.5,0.5],'Color',[0.7,0.7,0.7],'Linewidth',0.1)
    hold off
    axis equal
    xlim([-2,2]);
    ylim([-0.3,0.3]);
    drawnow
    if saveanim
      ax = gca;
      ax.Units = 'pixels';
      pos = ax.Position;
      aa = get(gcf);
      marg = 30;
      rect = [-marg, -marg, pos(3)+2*marg, pos(4)+2*marg];
      F = getframe(gca,rect);
      writeVideo(v,F);
    end
    pause(slowmo*framedel)
  end

  if saveanim
    close(v);
  end
end

function Xd = dynamics(params, t, X)

  Xd = zeros(size(X));

  th = X(1);
  phi = X(2);
  dth = X(3);
  dphi = X(4);

  % STUDENT CAN USE THIS TO TEST THEIR CONTROLLER
  z = [sin(phi)
    cos(phi);
    dphi];

  u = controllerNoisyEnc(params, t, z, th, dth);

  Xd(1:2) = X(3:4);
  % STUDENT COMPLETES THIS
  Xd(3:4) = eom(params, th, phi, dth, dphi, u);
end


function circle(x,y,r)
%x and y are the coordinates of the center of the circle
%r is the radius of the circle
%0.01 is the angle step, bigger values will draw the circle faster but
%you might notice imperfections (not very smooth)
ang = 0:0.01:2*pi;

%ang = [ang1,ang2,ang3,ang4,ang5];
xp=r*cos(ang);
yp=r*sin(ang);
plot(x+xp,y+yp,'k','LineWidth',2);
end

% TRAJECTORIES TO TRACK
function x = trajStep(t)
  % for scalar t
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
function x = trajSin(t)
  x = sin(t);
end
