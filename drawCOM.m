function [ h ] = drawCOM( com, h )

theta=linspace(0,2*pi,101);
if isempty(h)
    h{1}=plot(com(1)+cos(theta),com(2)+sin(theta),'k','linewidth',1);
    theta=linspace(0,pi/2,51);
    h{2}=patch(com(1)+[0,cos(theta),0],com(2)+[0,sin(theta),0],'k');
    theta=linspace(pi,3*pi/2,51);
    h{3}=patch(com(1)+[0,cos(theta),0],com(2)+[0,sin(theta),0],'k');
else
    delete(h{1});
    delete(h{2});
    delete(h{3});
    h{1}=plot(com(1)+cos(theta),com(2)+sin(theta),'k','linewidth',1);
    theta=linspace(0,pi/2,51);
    h{2}=patch(com(1)+[0,cos(theta),0],com(2)+[0,sin(theta),0],'k');
    theta=linspace(pi,3*pi/2,51);
    h{3}=patch(com(1)+[0,cos(theta),0],com(2)+[0,sin(theta),0],'k');
end


end

