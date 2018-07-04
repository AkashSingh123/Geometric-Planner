function [ h ] = drawSnake( hx, hy, ht, alpha, L )
n=size(alpha,1)+1;
h=cell(n);%link handle
g=cell(n);
h{1}=drawEllipse(hx,hy,ht,L);
g{1}=[cos(ht),-sin(ht),hx;sin(ht),cos(ht),hy;0,0,1];%module 0 configuration
F=[eye(2),[L;0];0 0 1];%move forward
R=@(a)[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];%rotation frame
for i=2:n
    %new module frame
    g{i}=g{i-1}*F*R(alpha(i-1))*F;
    h{i}=drawEllipse(g{i}(1,3),g{i}(2,3),ht+sum(alpha(1:i-1)),L);
end

