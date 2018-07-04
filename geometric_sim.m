%geometric_sim
addpath arrow3d/;
figure();hold on;axis equal;set(gcf,'color','w');
axis([-20 60 -40 40]);
omega_s=2*pi/8;
N=17;%number of modules
L=2;
Am=1.2;
set(gca,'xtick',-20:40:60,'ytick',-40:40:40);

alpha=@(n, t) Am*sin(omega_s*n+t);
d_alpha=@(n, t) Am*cos(omega_s*n+t);
conf_list=[];%configuration list; storing all the nodes
%start simulation part
activation=zeros(1,N);

t=0;%current time
T=100;%number of time steps
dt=2*pi/T;

joint_index=(1:N-1)';%joint index
q_h=[0;0;-pi/2+pi/10];
g_h=[cos(q_h(3)),-sin(q_h(3)),q_h(1);sin(q_h(3)),cos(q_h(3)),q_h(2);0,0,1];
[h, com]=drawActiveSnake(q_h(1),q_h(2),q_h(3),alpha(joint_index,t),2,activation);



%%
%visualization
com_path=com;%record center of mass trajecotry
h_com=plot(com_path(1,:),com_path(2,:),'b','linewidth',3);
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

desired_xi=[10;0;0];
title('Isotropic Friction with Lifting','fontsize',20);
xlabel('X','fontsize',15);
ylabel('Y','fontsize',15);
K=diag([1;1;50]);%friction profile

while t<2*pi

    %generate all feasiable contacts
    numContacts = 4;
    contact_list=feasibleContact2(alpha(joint_index,conf.t),L,numContacts);
    contacts=optimalContact(alpha(joint_index,conf.t), alpha(joint_index, conf.t+dt), d_alpha(joint_index,t),contact_list, desired_xi, dt, L, K);
    activation=zeros(1,N);
    activation(contact_list(:,contacts))=1;

%     activation=ones(1,N);
    
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
        for i=1:N
            delete(h{i});
        end
        
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



























































