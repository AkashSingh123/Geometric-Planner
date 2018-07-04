function [ h ] = drawActiveFrameEllipse( g, L, activation )

beta=linspace(0,2*pi,101);
e_x=L*cos(beta);
e_y=0.3*L*sin(beta);
e_p=[e_x;e_y];
R=g(1:2,1:2);
e_p=R*e_p;
if activation
    h=patch(g(1,3)+e_p(1,:),g(2,3)+e_p(2,:),'r');
else
    h=plot(g(1,3)+e_p(1,:),g(2,3)+e_p(2,:),'k','linewidth',1);
end

end

