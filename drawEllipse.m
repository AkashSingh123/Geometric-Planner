function [ h ] = drawEllipse( x, y, theta, L )

beta=linspace(0,2*pi,101);
e_x=L*cos(beta);
e_y=0.3*L*sin(beta);
e_p=[e_x;e_y];
R=[cos(theta),-sin(theta);sin(theta),cos(theta)];
e_p=R*e_p;
h=plot(x+e_p(1,:),y+e_p(2,:),'k','linewidth',0.5);

end

