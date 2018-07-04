function [ J ] = spatialJacobian( alpha, L )
%alpha are joint angles
%spatial Jacobian computes the spatial manipulator Jacobian with the head
%module defined as the spatial reference frame
%alpha: n X 1, J: 3 X n

%compute joint frame position: note the first frame is the head not the
%joint
n=size(alpha,1)+1;
q=zeros(2,n-1);%joint positions
g_joint=jointsInHead(alpha,L);
for i=1:n-1
    q(:,i)=g_joint{i+1}(1:2,3);
end
%get all the joint positions, and we now the rotational axis is [0 0 1]
%now, we construct the spatial jacobian
q=[q(2,:);-q(1,:)];%position part in the spatial Jacobian
J=[q;ones(1,n-1)];

end

