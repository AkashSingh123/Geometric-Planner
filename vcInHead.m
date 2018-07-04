function [ g_vc ] = vcInHead( alpha, L )
g=framesInHead(alpha,L);%compute all the module frames
com_in_head=computeCOM(g);
theta_in_head=mean(cumsum(alpha));
g_vc=[[cos(theta_in_head),-sin(theta_in_head);
        sin(theta_in_head),cos(theta_in_head)],com_in_head;0 0 1];%vc in head

end

