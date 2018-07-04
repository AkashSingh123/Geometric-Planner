function [ h, com ] = drawActiveSnake( hx, hy, ht, alpha, L, activation )
n=size(alpha,1)+1;
h=cell(1,n);%link handle
g=cell(1,n);
if activation(1)
    h{1}=drawActiveEllipse(hx,hy,ht,L,1);
else
    h{1}=drawActiveEllipse(hx,hy,ht,L,1);
end
g{1}=[cos(ht),-sin(ht),hx;sin(ht),cos(ht),hy;0,0,1];%module 0 configuration
F=[eye(2),[L;0];0 0 1];%move forward
R=@(a)[cos(a),-sin(a),0;sin(a),cos(a),0;0,0,1];%rotation frame
for i=2:n
    %new module frame
    g{i}=g{i-1}*F*R(alpha(i-1))*F;
    if activation(i)
        h{i}=drawActiveEllipse(g{i}(1,3),g{i}(2,3),ht+sum(alpha(1:i-1)),L,1);
    else
        h{i}=drawEllipse(g{i}(1,3),g{i}(2,3),ht+sum(alpha(1:i-1)),L);
    end
end
%scatter(g{17}(1,3),g{17}(2,3));
com=computeCOM(g);

end

