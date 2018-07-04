function next_headang = find_nex_headang_cir(g_h,m,n,g,L)
path=[m',n'];
F=[eye(2),[L;0];0 0 1];
neck=g_h*F;
neck_position=[neck(1,3),neck(2,3)];
tip =g_h*inv(F);
theta=atan(g_h(1,1)/g_h(2,1));
t = linspace(-pi/2+theta,pi/2+theta);plot((cos(t)/0.24+neck(1,3)),(sin(t)/0.24+neck(2,3)));
circle_cood=[(cos(t)/0.24+g_h(1,3))',(sin(t)/0.24+g_h(2,3))'];
tip_pos=[tip(1,3),tip(2,3)];
dist_matrix=pdist2(circle_cood,path);

[num idx]=min(dist_matrix(:));
[x,y]=ind2sub(size(dist_matrix),idx);
disp((ind2sub(size(dist_matrix),idx)));
if(size(y)>1)
    y=y(size(y));
end
target_point=path(y,:);

scatter(target_point(1),target_point(2));
%scatter(target_point(1),target_point(2));
%disp(target_point(1),target_point(2));

target_vector = target_point-neck_position;

% h=[neck(1,3),neck(1,3)+target_vector(1)];
% i=[neck(2,3),neck(2,3)+target_vector(2)];
%plot(h,i);
neck_link_conf = g_h*g{2};
neck_link_vect=[neck_link_conf(1,1),neck_link_conf(2,1)];
next_headang=-deviation_angle(-neck_link_vect,target_vector);
end