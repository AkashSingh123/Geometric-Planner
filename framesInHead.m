function [ g ] = framesInHead( alpha, L )
n=size(alpha,1)+1;
g=cell(1,n);

g{1}=eye(3);%module 0 configuration: identity in itself
F=[eye(2),[L;0];0 0 1];%move forward
R=@(a)[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];%rotation frame
for i=2:n
    %new module frame
    g{i}=g{i-1}*F*R(alpha(i-1))*F;
end

end

