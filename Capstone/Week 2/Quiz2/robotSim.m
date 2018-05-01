
function robotSim()
  % This file is starter code
  %system constants
  params = struct();
  params.m = 1;
  params.l = 0.5;
  params.g = 9.81;
  params.traj = @trajSin;

  tspan=[0,10];
  y0=[0,1,0,0];
  [t,X] = ode45(@(t,y) dynamics(params,t,y),tspan,y0);
  % size(X)

  % close('all')
  figure(1)
  subplot(3,1,1)
  hold all
  plot(t, X(:,1))
  plot(t, X(:,2))
  hold off
  ylabel('$\theta$ (rad)','Interpreter','latex')
  subplot(3,1,2)
  hold all
  plot(t, X(:,3))
  plot(t, X(:,4))
  hold off
  ylabel('$\dot\theta$ (rad/s)','Interpreter','latex')
  subplot(3,1,3)
  plot(t, X(:,1))
  ylabel('$\theta$ (rad)','Interpreter','latex')
  xlabel('t (sec)')

  % animate
  framedel = 0.05;
  slowmo = 0;
  tdraw = 0:framedel:t(end);
  Xdraw = interp1(t, X, tdraw);
  l = params.l;
  th1 = Xdraw(:,1);
  th2 = Xdraw(:,2);
  dth1 = Xdraw(:,3);
  dth2 = Xdraw(:,4);
  p1 = l*[cos(th1),sin(th1)];
  p2 = p1 + l*[cos(th1+th2),sin(th1+th2)];

  saveanim = 0;

  if saveanim
    v = VideoWriter('anim.avi','Uncompressed AVI');
    open(v);
  end

  % size(Xdraw)
  for i=1:numel(tdraw)
    figure(2)
    clf
    hold all
    % body
    line([0,p1(i,1),p2(i,1)],[0,p1(i,2),p2(i,2)],'Color',[.5,.5,.5],'Linewidth',3);
    % path
    plot(p2(1:i,1),p2(1:i,2))
    % traj
    trajNow = params.traj(tdraw(i));
    plot(trajNow(1),trajNow(2),'*')
    % t
    text(-0.1,1.5,['t = ',num2str(tdraw(i))])
    hold off
    axis equal
    xlim([-2,2]);
    ylim([-2,2]);
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

  th1 = X(1);
  th2 = X(2);
  dth1 = X(3);
  dth2 = X(4);

  u = controller(params, t, X);
  assert(numel(u)==2, 'output from controller should be a 2-element vector')
  u1 = -u(1);
  u2 = -u(2);

  m = params.m;
  l = params.l;
  g = params.g;

  Xd(1:2) = X(3:4);
  Xd(3:4) = [(-1).*l.^(-2).*m.^(-1).*((-2)+cos(th2).^2).^(-1).*((-1).*u1+u2+( ...
  -2).*g.*l.*m.*cos(th1)+u2.*cos(th2)+g.*l.*m.*cos(th2).*cos(th1+ ...
  th2)+2.*dth1.*dth2.*l.^2.*m.*sin(th2)+dth2.^2.*l.^2.*m.*sin(th2)+ ...
  dth1.^2.*l.^2.*m.*(1+cos(th2)).*sin(th2)),l.^(-2).*m.^(-1).*((-2)+ ...
  cos(th2).^2).^(-1).*((-1).*u1+3.*u2+(-2).*g.*l.*m.*cos(th1)+(-1).* ...
  u1.*cos(th2)+2.*u2.*cos(th2)+(-2).*g.*l.*m.*cos(th1).*cos(th2)+2.* ...
  g.*l.*m.*cos(th1+th2)+g.*l.*m.*cos(th2).*cos(th1+th2)+dth2.^2.* ...
  l.^2.*m.*(1+cos(th2)).*sin(th2)+dth1.*dth2.*l.^2.*m.*csc((1/2).* ...
  th2).^2.*sin(th2).^3+dth1.^2.*l.^2.*m.*(3.*sin(th2)+sin(2.*th2))) ...
  ];

end

function x = trajSin(t)
  x = 0.5*[cos(pi*t); sin(pi*t)];
end

function x = trajSquare(t)
  spd = 0.2;
  if t<2.5
    x = [0.5;0] + spd * [-1; 1] * (t);
  elseif t<5
    x = [0;0.5] + spd * [-1; -1] * (t-2.5);
  elseif t<7.5
    x = [-0.5;0] + spd * [1; -1] * (t-5);
  else
    x = [0;-0.5] + spd * [1; 1] * (t-7.5);
  end
end
