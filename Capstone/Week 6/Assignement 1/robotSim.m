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
  tsim = 2;
  y0=[0,1,0,0]';
  timestep = 0.001;
  dispstat('','init')

  for l=0:timestep:tsim-timestep
    dispstat(sprintf('t = %.3fs (out of %d)',l+timestep,tsim)); 
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

  close('all')
  figure(1)
  subplot(3,1,1)
  plot(fullt, X(:,2))
  ylabel('$\phi$ (rad)','Interpreter','latex')
  subplot(3,1,2)
  plot(fullt, X(:,4))
  ylabel('$\dot\phi$ (rad/s)','Interpreter','latex')
  subplot(3,1,3)
  plot(fullt, X(:,1))
  ylabel('$\theta$ (rad)','Interpreter','latex')
  xlabel('t (sec)')

  % animate
  framedel = 0.02;
  slowmo = 0;
  tdraw = 0:framedel:fullt(end);
  Xdraw = interp1(fullt, X, tdraw);
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
    pause(slowmo*framedel)
  end
  
  
  clear all
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

  u = controllerNoisy(params, t, z);

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

