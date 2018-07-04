%geometric_sim
addpath arrow3d/;
x_min = -60;
x_max = 150;
y_min = -50;
y_max = 50;
t_min=0;
t_max=2*22*pi;
d_min=0;
d_max=61;
com3=0;
com2=0;
figure();hold on;axis equal;set(gcf,'color','w');
axis([x_min x_max y_min y_max]);
recording = 0;
if recording
    writerObj = VideoWriter('Vid15');
    open(writerObj);
end

omega_s=2*pi/8;
N=17;%number of modules
L=2;
Am=1.2;
set(gca,'xtick',x_min:20:x_max,'ytick',y_min:20:y_max);


%alpha = [0;pi/2;0;-pi/2;0;pi/2;0;-pi/2;0;pi/2;0;-pi/2;0;pi/2;0;-pi/2;0];
angle1=Am*sin(omega_s*(1:16)');


%alpha=@(n, t) Am*sin(omega_s*n+t);
%alpha(n,t) = Am*sin(omega_s*n+t);

d_alpha=@(n, t) Am*cos(omega_s*n+t);
conf_list=[];%configuration list; storing all the nodes
%start simulation part
activation=zeros(1,N);

t=0;%current time
T=100;%number of time steps
dt=2*pi/T;

joint_index=(1:N-1)';%joint index
q_h=[-57;-13.9;-pi/2+.04];
g_h=[cos(q_h(3)),-sin(q_h(3)),q_h(1);sin(q_h(3)),cos(q_h(3)),q_h(2);0,0,1];
[h, com]=drawActiveSnake(q_h(1),q_h(2),q_h(3),angle1,2,activation);
F=[eye(2),[L;0];0 0 1];

counter = 1;

[peg_X,peg_Y] = meshgrid(x_min:7:x_max,y_min:7:y_max);
peg_X = peg_X(:);
peg_Y = peg_Y(:);
scatter(peg_X,peg_Y,10,'r','fill');
plot(peg_X,peg_Y,'.');
voronoi(peg_X,peg_Y);
Distance_travelled_percycle = zeros(20,13);
%g=framesInHead(alpha(joint_index, 0),L);
%position_lastlink=g_h*g{17}-position_lastlink;

%   x=rand(100,1)*5;
%   y=rand(100,1)*5;
%   plot(x,y,'.')
%   voronoi(x,y);
% [workspace_X,workspace_Y] = meshgrid(-20:60,-40:40);
% K_height = 0*workspace_X;
%
% for i = 1:size(workspace_X,1)
%    for j = 1:size(workspace_X,2)
%       [~,k] = generate_K_map(workspace_X(i,j),workspace_Y(i,j),15);
%      K_height(i,j) = k;
%    end
% end
% surf(workspace_X,workspace_Y,K_height);
% zlabel('anisotropic friction coefficient K','fontsize',20)
%
% for i= -20:20:60
% m(i+21,:)=linspace(i,i,10);
% p(i+21,:)=linspace(-40,40,10);
% scatter(m(i+21,:),p(i+21,:),10,'r','fill');
% end
%
%
% Kvalue = generate_K_map(1,2,m,p);
% disp(Kvalue);
%generate_Kmap;
%%
%visualization
com_path=com;%record center of mass trajecotry
h_com=plot(com_path(1,:),com_path(2,:),'b','linewidth',3);
%%
t=0;
fi=0;
f_num=0;

p_com=[];
%initilize conf
conf.t=0;%set up time
conf.g=g_h*vcInHead(angle1,L);%vc position
conf.contacts=[];%contacts: empty at the very beginning
conf.contact_cost=0;%no cost at the beginning
conf.heuristic_cost=heuristicCost(conf.g);%heuristic cost
conf.estimated_cost=conf.contact_cost+conf.heuristic_cost;

title('pegspacing/wavelength=0.93');
xlabel('X','fontsize',15);
ylabel('Y','fontsize',15);
K=diag([1;4;1]);%friction profile
activation=ones(1,N);

%d_alpha = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
%generateuniform_peg(figure);
com3 = com;
%figure();hold on;axis equal;set(gcf,'color','w');
%axis([t_min t_max d_min d_max]);
%set(gca,'xtick',t_min:2*11*pi:t_max,'ytick',d_min:30:d_max);
peg_spacing=7;
angle_buffer=Am*sin(omega_s*(1:1/20:16));
buffer=Init_Buffer(angle_buffer');
while t<2*pi*50
    [buffer,angle2]=find_angles(buffer,t);
    diff_angles = angle2-angle1;
    angle1=angle2;
    %position_lastlink=g_h*g{17};
    %test
    %peg_distance = peg_distance+1;
    if recording
        frame = getframe(gcf);
        writeVideo(writerObj,frame);
    end
    %conf=forwardSim(conf, alpha(joint_index, conf.t), alpha(joint_index, conf.t+dt), d_alpha(joint_index, conf.t), activation, dt, L,  K ,g_h,peg_spacing);
    conf=forwardSim(conf, angle1, angle2, diff_angles, activation, dt, L,  K ,g_h,peg_spacing);
    %%
    %compute vc in head
    t=t+dt;
    %vc_in_head=vcInHead(alpha(joint_index, conf.t), L);
    vc_in_head=vcInHead(angle2, L);
    g_h=conf.g/vc_in_head;%move the head
    q_h=[g_h(1,3);g_h(2,3);atan2(g_h(2,1),g_h(1,1))];
    
    g=framesInHead(angle2,L);
    %
    %chconfig_lastlink = (g_h*g{17}-position_lastlink);
    %velocity_lastlink = [chconfig_lastlink(1,3),chconfig_lastlink(2,3)];
    %disp( velocity_lastlink);
    no =(1:22);
    fi=fi+1;
    if rem(fi,1)==0
        %redraw the snake
        for i=1:N
            delete(h{i});
        end
        
        %draw at discrete frames
        [h,com]=drawActiveFrameSnake(g_h,angle2,L,activation);
        com_path=[com_path,com];%#ok
        p_com=drawCOM(com,p_com);
        set(h_com,'xdata',com_path(1,:),'ydata',com_path(2,:));
        f_num=f_num+1;
        %         print(gcf,['record/',num2str(f_num,'%03d'),'.png'],'-dpng');
        pause(0.5*dt);
    else
        
        
    end
    %com1=com;
    
    if any(no*2*pi -dt/2<t & no*2*pi +dt/2>t)
        %  peg_spacing = peg_spacing+5;
        %  q_h=[-60;0;-pi/2];
        %  g_h=[cos(q_h(3)),-sin(q_h(3)),q_h(1);sin(q_h(3)),cos(q_h(3)),q_h(2);0,0,1];
        %counter = counter+1;
        com2=com;
        disp(com2)
        Distance_travelled_percycle(counter,peg_spacing-2) = norm(com2-com3);
        counter = counter + 1;
        % scatter(t,norm(Distance_travelled_percycle),50,'fill','r');
        %disp(Distance_travelled_percycle);
        %disp(com2(1)-com3(1));
        com3=com2;
        
    end
    %scatter(Distance_travelled_percycle,d(6));
    %frame=getframe(gcf);
    %writeVideo(writerObj,frame);
end
if recording
    close(writerObj);
end
% end
