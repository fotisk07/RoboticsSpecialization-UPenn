
function u = controller(params, t, X)
  % You have full state feedback available
  K=[-1.0001 -113.1849   -1.2465  -13.9740];
  u=-K*X;

  % After doing the steps in simLinearization, you should be able to substitute the linear controller u = -K*x
  
end

