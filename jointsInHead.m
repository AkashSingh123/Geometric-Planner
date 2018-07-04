function [ g ] = jointsInHead( alpha, L )
%compute all the joint frame, defined as the proximal end of the next
%module at the joint
n=size(alpha,1)+1;%number of modules
g=cell(1,n+1);
g{1}=eye(3);%put the head frame at the identity
F=[eye(2),[L;0];0 0 1];%move forward half link length
R=@(a)[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];
g{1}=g{1}/F;%the virtual 0th joint
%put 0 joint back

for i=2:n
    g{i}=g{i-1}*F*F*R(alpha(i-1));
end
g{n+1}=g{n}*F*F;%the tail frame

end

