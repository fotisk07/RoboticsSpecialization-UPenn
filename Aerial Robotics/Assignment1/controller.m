function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

kp=180;
kv=100;
u = params.mass*(kp*(s_des(1)-s(1))+kv*(s_des(2)-s(2))+params.gravity);

% FILL IN YOUR CODE HERE


end

