
function Xend = simode(beta, tend)
  % this parameter is set by the calling script
  params.delta = 1;
  params.alpha = 1;
  params.beta = beta;
  params.gamma = 1;
  params.omega = 1; 
  % this is the initial condition
  X0 = [1,1];
  ts=[0 ,tend];
  
  [~,X]=ode45(@(t,X) dyn(params,t,X) , ts , X0);
  Xend = X(end,:);
  
  % student fills this out
  % You should write code that integrates the ode from time 0 to tend, and assigns "Xend" such that Xend = X at time tend.
  
  

  % The following line should help plot the solutions if you assign to X the second output of ode45
  % plot(X(:,1), X(:,2))
end

function Xd = dyn(params, t, X)
  x = X(1);
  xd = X(2);
  xd = X(2);

  Xd = zeros(size(X));
  
  Xd(1) = xd;
  Xd(2) = params.gamma*cos(params.omega*t)-params.delta*xd - params.alpha*x - params.beta*x^3;

  % student completes this
  % Note: you have all the parameters available here, e.g. params.alpha
  
end

