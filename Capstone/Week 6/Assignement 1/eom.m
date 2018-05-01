function qdd = eom(params, th, phi, dth, dphi, u)
  % This is the starter file for the week5 assignment

  % Provided params are
  % params.g: gravitational constant
  % params.mr: mass of the "rod"
  % params.ir: rotational inertia of the rod
  % params.d: distance of rod CoM from the wheel axis
  % params.r: wheel radius

  % Provided states are:
  % th: wheel angle (relative to body)
  % phi: body pitch
  % dth, dphi: time-derivatives of above
  % u: torque applied at the wheel
  
    r = params.r;
  g = params.g;
  mb = params.mr;
  m = mb;
  i = params.ir;
  l = params.d;
  theta = th;
  phidot = dphi;
  tdot = dth;

  A = [ mb*r^2,                                                        mb*r*(r + l*cos(phi));
     (mb*r*(2*r + 2*l*cos(phi)))/2,         i + (mb*((2*r + 2*l*cos(phi))*(r + l*cos(phi)) + 2*l^2*sin(phi)^2))/2];  
  

 B =  [                                                                                                                                                                                                               l*mb*r*sin(phi)*phidot^2 + u;
 (mb*(2*l*sin(phi)*phidot*(r*(phidot + tdot) + l*cos(phi)*phidot) - 4*l^2*cos(phi)*sin(phi)*phidot^2 + l*sin(phi)*phidot^2*(2*r + 2*l*cos(phi))))/2 + (mb*(2*l^2*cos(phi)*sin(phi)*phidot^2 - 2*l*sin(phi)*phidot*(r*(phidot + tdot) + l*cos(phi)*phidot)))/2 + g*l*mb*sin(phi)];   

 qdd = linsolve(A,B);
  % THE STUDENT WILL FILL THIS OUT
end