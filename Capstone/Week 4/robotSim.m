
function robotSim()
  % This file is starter code
  %system constants
  params = struct();

  params.g = 9.81;
  params.mr = 0.25;
  params.ir = 0.0001;
  params.d = 0.1;
  params.r = 0.02;

  tspan=[0,2];
  y0=[0,1,0,0];
  [t,X] = ode45(@(t,y) dynamics(params,t,y),tspan,y0);
  % size(X)

  % close('all')
  figure(1)
  subplot(3,1,1)
  plot(t, X(:,2))
  ylabel('$\phi$ (rad)','Interpreter','latex')
  subplot(3,1,2)
  plot(t, X(:,4))
  ylabel('$\dot\phi$ (rad/s)','Interpreter','latex')
  subplot(3,1,3)
  plot(t, X(:,1))
  ylabel('$\theta$ (rad)','Interpreter','latex')
  xlabel('t (sec)')

  % animate
  framedel = 0.02;
  slowmo = 0;
  tdraw = 0:framedel:t(end);
  Xdraw = interp1(t, X, tdraw);

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
    text(-0.1,-0.3,['t = ',num2str(tdraw(i))])
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

  % leave as 0 till next week
  u = 0;

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

