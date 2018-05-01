function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
H=H*sign(H(3,3));

R2=[H(:,1) H(:,2) cross(H(:,1),H(:,2))];
[u,d,v]=svd(R2);
R=u*[1 0 0;0 1 0;0 0 det(u*v')]*v';
t=H(:,3)/norm(H(:,1));


% YOUR CODE HERE: Project the points using the pose
N=size(render_points,1);
proj_points=zeros(N,2);

for i=1:N
    Xc=K*(R*render_points(i,:)'+t);
    Xc=Xc/Xc(3);
    proj_points(i,:)=[Xc(1) Xc(2)];
end

end
