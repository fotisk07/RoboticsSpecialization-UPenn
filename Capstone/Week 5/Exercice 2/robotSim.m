
function robotSim()
  % This file is starter code
  %system constants
  params = struct();
  params.g = 9.81;
  params.mr = 0.25;
  params.ir = 0.0001;
  params.d = 0.1;
  params.r = 0.02;
  params.ubias = -1;

  tspan=[0,5];

  y0=[0,0.1,0,0];
  % Need this for euler integration inside the controller function
  options = odeset('MaxStep',0.001);
  [t,X] = ode45(@(t,y) dynamics(params,t,y),tspan,y0, options);
  % size(X)

  close('all')
  figure(1)
  subplot(3,1,1)
  hold all
  plot(t, X(:,2))
  plot([min(t),max(t)],[0,0],'k--')
  hold off
  ylabel('$\phi$ (rad)','Interpreter','latex')
  subplot(3,1,2)
  hold all
  plot(t, X(:,4))
  plot([min(t),max(t)],[0,0],'k--')
  hold off
  ylabel('$\dot\phi$ (rad/s)','Interpreter','latex')
  subplot(3,1,3)
  plot(t, X(:,1))
  ylabel('$\theta$ (rad)','Interpreter','latex')
  xlabel('t (sec)')

  % animate
  framedel = 0.05;
  slowmo = 0;
  tdraw = 0:framedel:t(end);
  Xdraw = interp1(t, X, tdraw);
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
    pause(slowmo*framedel)
  end
end

function [Xd,params] = dynamics(params, t, X)

  Xd = zeros(size(X));

  th = X(1);
  phi = X(2);
  dth = X(3);
  dphi = X(4);

  u = controller(params, t, phi, dphi);
  assert(isscalar(u), 'output from controller should be a scalar value')

  u = u + params.ubias;
  % u = max(min(u,umax),-umax);

  Xd(1:2) = X(3:4);
  % STUDENT COMPLETES THIS
  Xd(3:4) = eom(params, th, phi, dth, dphi, u);
  % Could add wheel dammping here
  % Xd(3) = Xd(3) - 1*dth;
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

