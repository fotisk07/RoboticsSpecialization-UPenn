function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

kpz =180;
kvz =80;

kpy=10;
kvy=10;

kpf=140;
kvf=17;

u1=params.mass*(params.gravity + des_state.acc(2) + kvz*(des_state.vel(2)-state.vel(2)) + kpz*(des_state.pos(2)-state.pos(2)));
phic= -1/params.gravity * ( des_state.acc(1) + kvy*(des_state.vel(1)-state.vel(1))+kpy*(des_state.pos(1)-state.pos(1)));
u2= params.Ixx*( kvf*(0-state.omega) + kpf*(phic - state.rot));

if u1> params.maxF
    u1=params.maxF;
end
    
if u1<params.minF
    u1=params.minF;
end
    

% FILL IN YOUR CODE HERE

end

