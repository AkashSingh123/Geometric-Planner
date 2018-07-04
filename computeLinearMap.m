function [ A ] = computeLinearMap( alpha, L, activation, K )
%compute the Jacobian like matrix which maps shape velocity to body
%velocity (in the head frame)
n=size(alpha,1)+1;
g=framesInHead(alpha,L);%get all the modules frame relative to head
% k=10;
% K=diag([k k 200*k]);
%compute omega1
omega1=forceMapping(g{1})*K*activation(1);
for i=2:n
    omega1=omega1+activation(i)*forceMapping(g{i})*K*adjoint(inv(g{i}));
end
%compute omega2
%compute spatial Jacobian
J=spatialJacobian(alpha,L);
omega2=zeros(3,n-1);
for i=2:n
    omega2=omega2+activation(i)*forceMapping(g{i})*K*adjoint(inv(g{i}))*[J(:,1:i-1),zeros(3,n-i)];
end
epsilon=5;
[u,s,v]=svd(omega1);
omega1=u*(s+epsilon*eye(3))*v';
A=-omega1\omega2;

end

