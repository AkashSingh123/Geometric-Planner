%geometric_sim
addpath arrow3d/;
figure();hold on;axis equal;set(gcf,'color','w');
axis([-60 60 -40 40]);
omega_s=2*pi/8;
n=9;%number of modules
L=2;
As = 1.2;
Ac = 1.0;

alpha=@(n, t) As*sin(t).*sin(omega_s*n)+Ac*cos(t).*cos(omega_s*n);
d_alpha=@(n, t) As*cos(t).*sin(omega_s*n)-Ac*sin(t).*sin(omega_s*n);
conf_list=[];%configuration list; storing all the nodes
%start simulation part
activation=zeros(1,n);

t=0;%current time
T=100;%number of time steps
dt=2*pi/T;

joint_index=(1:n-1)';%joint index
q_h=[0;0;-pi/2+pi/10];
g_h=[cos(q_h(3)),-sin(q_h(3)),q_h(1);sin(q_h(3)),cos(q_h(3)),q_h(2);0,0,1];
[h, com]=drawActiveSnake(q_h(1),q_h(2),q_h(3),alpha(joint_index,t),2,activation);



%%
%visualization
com_path=com;%record center of mass trajecotry
h_com=plot(com_path(1,:),com_path(2,:),'b','linewidth',3);
h_vc_x=arrow([com(1),com(2)],...
    [com(1)+10*cos(q_h(3)+sum(cumsum(alpha(joint_index,t)))/n),...
    com(2)+10*sin(q_h(3)+sum(cumsum(alpha(joint_index,t)))/n)],10);
h_vc_y=arrow([com(1),com(2)],...
    [com(1)+10*cos(q_h(3)+pi/2+sum(cumsum(alpha(joint_index,t)))/n),...
    com(2)+10*sin(q_h(3)+pi/2+sum(cumsum(alpha(joint_index,t)))/n)],10);
%%

fi=0;
f_num=0;

p_com=[];
%initilize conf
conf.t=0;%set up time
conf.g=g_h*vcInHead(alpha(joint_index,conf.t),L);%vc position
conf.contacts=[];%contacts: empty at the very beginning
conf.contact_cost=0;%no cost at the beginning
conf.heuristic_cost=heuristicCost(conf.g);%heuristic cost
conf.estimated_cost=conf.contact_cost+conf.heuristic_cost;

desired_xi=[0;0;1];
title('Isotropic Friction with Lifting','fontsize',20);
xlabel('X','fontsize',15);
ylabel('Y','fontsize',15);
K=diag([1;1;1]);%friction profile

while t<2*pi
    activation=ones(1,n);
    
    %%
    %test
    conf=forwardSim(conf, alpha(joint_index, conf.t), alpha(joint_index, conf.t+dt), d_alpha(joint_index, conf.t), activation, dt, L, K);
    
    %%
    %compute vc in head
    t=t+dt;
    vc_in_head=vcInHead(alpha(joint_index, conf.t), L);
    g_h=conf.g/vc_in_head;%move the head
    q_h=[g_h(1,3);g_h(2,3);atan2(g_h(2,1),g_h(1,1))];
    
    fi=fi+1;
    if rem(fi,1)==0
        %redraw the snake
        for i=1:n
            delete(h{i});
        end
        delete(h_vc_x,h_vc_y);
        h_vc_x=arrow([com(1),com(2)],...
        [com(1)+10*cos(q_h(3)+sum(cumsum(alpha(joint_index,t)))/n),...
        com(2)+10*sin(q_h(3)+sum(cumsum(alpha(joint_index,t)))/n)],10);
        h_vc_y=arrow([com(1),com(2)],...
        [com(1)+10*cos(q_h(3)+pi/2+sum(cumsum(alpha(joint_index,t)))/n),...
        com(2)+10*sin(q_h(3)+pi/2+sum(cumsum(alpha(joint_index,t)))/n)],10);
        
        %draw at discrete frames
        [h,com]=drawActiveFrameSnake(g_h,alpha(joint_index,t),L,activation);
        com_path=[com_path,com];%#ok
        p_com=drawCOM(com,p_com);
        set(h_com,'xdata',com_path(1,:),'ydata',com_path(2,:));
        f_num=f_num+1;
%         print(gcf,['record/',num2str(f_num,'%03d'),'.png'],'-dpng');
        pause(0.5*dt);
    else
        
    end
end



























































