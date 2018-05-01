function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 1) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 1) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

n=size(x1,1);

P1=K*R1*[eye(3) -C1];
P2=K*R2*[eye(3) -C2];

p1=[x1 ones(n,1)];
p2=[x2 ones(n,1)];

X=[];

for i=1:n
    A=[cm(p1(i,:))*P1; cm(p2(i,:))*P2];
    [u,d,v]=svd(A);
    size(A)
    w=v(:,end)/v(end,end);
    w=reshape(w,1,4);
    X=[X;w];
end


X=X(:,1:3);


