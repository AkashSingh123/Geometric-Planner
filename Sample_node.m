
%geometric_sim
close all
clear
addpath arrow3d/;
x_min = -60;
x_max =120;
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
recording =1;
%%
if recording
    writerObj = VideoWriter('curvepath4_varparameters');
    open(writerObj);
end

%%
omega_s=2*pi/8;
N=17;%number of modules
L=2;
Am=1.2;
set(gca,'xtick',x_min:20:80,'ytick',y_min:20:y_max);


angle=Am*sin(omega_s*(1:N-1)');


alpha=@(t) Am*sin(t);

conf_list=[];%configuration list; storing all the nodes
%start simulation part
activation=zeros(1,N);

T=100;%number of time steps
dt=2*pi/T;

joint_index=(1:N-1)';%joint index
q_h=[-6.6;-6.8;pi/2+pi/10];
g_h=[cos(q_h(3)),-sin(q_h(3)),q_h(1);sin(q_h(3)),cos(q_h(3)),q_h(2);0,0,1];
[h, com]=drawActiveSnake(q_h(1),q_h(2),q_h(3),angle,2,activation);
F=[eye(2),[L;0];0 0 1];
peg_spacing=8;
[peg_X,peg_Y] = meshgrid(x_min:peg_spacing:7.0948,y_min:peg_spacing:y_max);
pegs_lineX = linspace(10.0948,100,13);
pegs_lineY = 4*ones(1,13); 
pegs_lineX1 = linspace(10.0948,100,13);
pegs_lineY1 = 20*ones(1,13); 

pegs_lineX2 = linspace(10.0948,100,13);
pegs_lineY2 = 5*ones(1,13);

pegs_lineX3 = linspace(10.0948,100,13);
pegs_lineY3 = -10*ones(1,13);

pegs_lineX4 = [pegs_lineX1,pegs_lineX2,pegs_lineX3];
pegs_lineY4 = [pegs_lineY1,pegs_lineY2,pegs_lineY3];
% scatter(pegs_lineX2,pegs_lineY2,10,'r','fill');
plot(pegs_lineX3,pegs_lineY3,'.');
voronoi(pegs_lineX4,pegs_lineY4);

[vx,vy]= voronoi(pegs_lineX4,pegs_lineY4);
