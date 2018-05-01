
function u = controller(params, t, phi, phidot)
  % STUDENT FILLS THIS OUT
  % 
  % Initialize any added state like this:
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

