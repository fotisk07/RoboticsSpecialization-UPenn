function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
Ax=[-video_pts -1*ones(4,1) zeros(4,3) video_pts.*(logo_pts(:,1)*ones(1,2)) logo_pts(:,1)];
Ay=[zeros(4,3) -video_pts -ones(4,1) video_pts.*(logo_pts(:,2)*ones(1,2)) logo_pts(:,2)];

A=[Ax(1,:);Ay(1,:);Ax(2,:);Ay(2,:);Ax(3,:);Ay(3,:);Ax(4,:);Ay(4,:)];




[U,S,V]=svd(A);

% disp('size V: '); size(V);


h=V(:,end);

% disp('size h: '); size(h)
H=reshape(h,[3,3]);

% disp('size H: '); size(H);


H =H';



end

