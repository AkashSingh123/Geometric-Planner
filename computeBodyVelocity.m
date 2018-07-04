function [ xi ] = computeBodyVelocity( alpha, d_alpha, L, activation, K,g_h,peg_spacing,vx,vy,r1,r2)
%compute the body velocity of the head frame
%compute all the frames relative to the head frame
n=size(alpha,1)+1;
g=framesInHead(alpha,L);%get all the modules frame relative to head

% k=10;
% K=diag([k k 200*k]);
%compute omega1
g_i=g_h;
[generated_K,~] = generate_K_map(g_i(1,3),g_i(2,3),peg_spacing,vx,vy,r1,r2);
omega1=(myAdjoint(inv(g{1}))).'*generated_K*activation(1);
for i=2:n
    g_i = g_h*g{i};
    [generated_K,~] = generate_K_map(g_i(1,3),g_i(2,3),peg_spacing,vx,vy,r1,r2);
    omega1=omega1+activation(i)*(myAdjoint(inv(g{i}))).'*generated_K*myAdjoint(inv(g{i}));
end
%compute omega2
%compute spatial Jacobian
J=spatialJacobian(alpha,L);
omega2=zeros(3,1);
for i=2:n
    g_h_i = g_h*g{i};
    generated_K = generate_K_map(g_h_i(1,3),g_h_i(2,3),peg_spacing,vx,vy,r1,r2);
    omega2=omega2+activation(i)*(myAdjoint(inv(g{i}))).'*generated_K*myAdjoint(inv(g{i}))*[J(:,1:i-1),zeros(3,n-i)]*d_alpha;
end

xi=-omega1\omega2;
end


