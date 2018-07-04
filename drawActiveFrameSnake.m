function [ h, com ] = drawActiveFrameSnake( g, alpha, L, activation )
n=size(alpha,1)+1;
h=cell(1,n);%link handle
if activation(1)
    h{1}=drawActiveFrameEllipse(g,L,1);
else
    h{1}=drawActiveFrameEllipse(g,L,0);
end
k=framesInHead(alpha,L);
k{1}=g;%update head frame
for i=2:n
    k{i}=g*k{i};%update module frame
    if activation(i)
        h{i}=drawActiveFrameEllipse(k{i},L,1);
    else
        h{i}=drawActiveFrameEllipse(k{i},L,0);
    end
end
com=computeCOM(k);

end

