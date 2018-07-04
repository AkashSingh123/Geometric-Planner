function [ h ] = drawActiveEllipse( x, y, theta, L, activation )

beta=linspace(0,2*pi,101);
e_x=L*cos(beta);
e_y=0.3*L*sin(beta);
e_p=[e_x;e_y];


if activation
    h=patch(x+e_p(1,:),y+e_p(2,:),'r');
else
    h=plot(x+e_p(1,:),y+e_p(2,:),'k','linewidth',0.5);
end

end

