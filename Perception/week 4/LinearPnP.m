function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

n=size(x,1);


x=[x ones(n,1)];
X=[X ones(n,1)];

b=inv(K)*x';
x=b';
% x=x/(x(:,end)*ones(1,3));

A=[];

for i=1:n
    a=[zeros(1,4) -X(i,:) x(i,2)*X(i,:);
        X(i,:) zeros(1,4) -x(i,1)*X(i,:);
        -x(i,2)*X(i,:) x(i,1)*X(i,:) zeros(1,4)];
    A=[A;a];
end


[u,d,v]=svd(A);
P=reshape(v(:,end),4,3);
P=P';

R=P(:,1:3);
t=P(:,end);

[u,d,v]=svd(R);
if det(u*v') > 0
    R=u*v';
    t=t/d(1,1);
else
   R=-u*v';
   t=-t/d(1,1);

end

C=-R'*t;




